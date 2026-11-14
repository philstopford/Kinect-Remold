#include "ScannerVisualTracker.hpp"
#include "RgbdRegistration.hpp"

#include <QQuaternion>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace {
constexpr int stride=4;
struct Cell {quint8 gray=0;QVector3D point;bool valid=false;};
struct Grid {int width=0,height=0;std::vector<Cell> cells;
  const Cell& at(int x,int y) const {return cells[size_t(y)*width+x];}
};
struct Feature {int x,y;float score;};
struct Pair {QVector3D current,reference;};

Grid registered(const QByteArray& depth,const DepthIntrinsics& camera,const QImage& color) {
  Grid grid;
  if(color.isNull()||!camera.valid()||depth.size()!=qsizetype(camera.width)*camera.height*2)return grid;
  grid.width=camera.width/stride;grid.height=camera.height/stride;
  grid.cells.resize(size_t(grid.width)*grid.height);
  const bool aligned=color.width()==camera.width&&color.height()==camera.height;
  for(int gy=0;gy<grid.height;++gy)for(int gx=0;gx<grid.width;++gx) {
    const int x=gx*stride+stride/2,y=gy*stride+stride/2;
    const int offset=(y*camera.width+x)*2;
    const quint16 mm=quint8(depth[offset])|(quint16(quint8(depth[offset+1]))<<8);
    if(mm<350||mm>4000)continue;
    float u=0,v=0;
    if(aligned){u=float(x);v=float(y);}
    else if(!RgbdRegistration::projectKinectOne(camera,x,y,mm,color.width(),color.height(),u,v))continue;
    if(u<1||v<1||u>=color.width()-1||v>=color.height()-1)continue;
    double rx=0,ry=0;if(!camera.ray(x,y,rx,ry))continue;
    Cell& cell=grid.cells[size_t(gy)*grid.width+gx];
    cell.gray=quint8(qGray(color.pixel(int(std::lround(u)),int(std::lround(v)))));
    cell.point={float(rx)*mm*.001f,float(ry)*mm*.001f,mm*.001f};cell.valid=true;
  }
  return grid;
}

std::vector<Feature> features(const Grid& grid) {
  std::vector<Feature> candidates,selected;
  if(grid.width<10||grid.height<10)return selected;
  for(int y=3;y<grid.height-3;++y)for(int x=3;x<grid.width-3;++x) {
    float xx=0,xy=0,yy=0;bool valid=true;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      const auto& a=grid.at(x+dx-1,y+dy),&b=grid.at(x+dx+1,y+dy);
      const auto& c=grid.at(x+dx,y+dy-1),&d=grid.at(x+dx,y+dy+1);
      if(!a.valid||!b.valid||!c.valid||!d.valid){valid=false;break;}
      const float gx=float(b.gray)-a.gray,gy=float(d.gray)-c.gray;
      xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;
    }
    if(!valid)continue;
    const float trace=xx+yy,det=xx*yy-xy*xy;
    const float score=det-.04f*trace*trace;
    if(score>100000.f)candidates.push_back({x,y,score});
  }
  std::sort(candidates.begin(),candidates.end(),[](const Feature& a,const Feature& b){
    return a.score>b.score;
  });
  for(const Feature& feature:candidates) {
    bool near=false;
    for(const Feature& previous:selected)if(std::abs(previous.x-feature.x)<3&&
        std::abs(previous.y-feature.y)<3){near=true;break;}
    if(!near)selected.push_back(feature);
    if(selected.size()>=120)break;
  }
  return selected;
}

bool patch(const Grid& grid,int x,int y,std::array<float,25>& values,float& variance) {
  float sum=0;int index=0;
  for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx) {
    const Cell& cell=grid.at(x+dx,y+dy);
    if(!cell.valid)return false;
    values[size_t(index++)]=cell.gray;sum+=cell.gray;
  }
  const float mean=sum/25;variance=0;
  for(float& value:values){value-=mean;variance+=value*value;}
  return variance>200;
}

std::vector<Pair> match(const Grid& current,const Grid& reference) {
  std::vector<Pair> pairs;
  if(current.width!=reference.width||current.height!=reference.height)return pairs;
  const auto candidates=features(current);
  for(const Feature& feature:candidates) {
    std::array<float,25> sample{};float sampleVariance=0;
    if(!patch(current,feature.x,feature.y,sample,sampleVariance))continue;
    float best=-1;int bestX=-1,bestY=-1;
    for(int y=std::max(3,feature.y-18);y<std::min(reference.height-3,feature.y+19);++y)
      for(int x=std::max(3,feature.x-18);x<std::min(reference.width-3,feature.x+19);++x) {
        const Cell& other=reference.at(x,y);
        if(!other.valid||std::abs(other.point.z()-current.at(feature.x,feature.y).point.z())>.35f)continue;
        std::array<float,25> candidate{};float variance=0;
        if(!patch(reference,x,y,candidate,variance))continue;
        float dot=0;
        for(int i=0;i<25;++i)dot+=sample[size_t(i)]*candidate[size_t(i)];
        const float score=dot/std::sqrt(sampleVariance*variance);
        if(score>best){best=score;bestX=x;bestY=y;}
      }
    if(best<.70f||bestX<0)continue;
    pairs.push_back({current.at(feature.x,feature.y).point,
                     reference.at(bestX,bestY).point});
  }
  return pairs;
}

bool rigid(const std::vector<Pair>& pairs,const std::vector<int>& indices,ScannerPose& pose) {
  if(indices.size()<3)return false;
  QVector3D pc,qc;
  for(int index:indices){pc+=pairs[size_t(index)].current;qc+=pairs[size_t(index)].reference;}
  pc/=float(indices.size());qc/=float(indices.size());
  double h[3][3]{};
  for(int index:indices) {
    const QVector3D p=pairs[size_t(index)].current-pc,q=pairs[size_t(index)].reference-qc;
    for(int row=0;row<3;++row)for(int col=0;col<3;++col)h[row][col]+=double(p[row])*q[col];
  }
  const double trace=h[0][0]+h[1][1]+h[2][2];
  double n[4][4]{{trace,h[1][2]-h[2][1],h[2][0]-h[0][2],h[0][1]-h[1][0]},
      {h[1][2]-h[2][1],h[0][0]-h[1][1]-h[2][2],h[0][1]+h[1][0],h[2][0]+h[0][2]},
      {h[2][0]-h[0][2],h[0][1]+h[1][0],-h[0][0]+h[1][1]-h[2][2],h[1][2]+h[2][1]},
      {h[0][1]-h[1][0],h[2][0]+h[0][2],h[1][2]+h[2][1],-h[0][0]-h[1][1]+h[2][2]}};
  double vectors[4][4]{};for(int i=0;i<4;++i)vectors[i][i]=1;
  for(int iteration=0;iteration<40;++iteration) {
    int p=0,q=1;double largest=0;
    for(int row=0;row<4;++row)for(int col=row+1;col<4;++col)
      if(std::abs(n[row][col])>largest){largest=std::abs(n[row][col]);p=row;q=col;}
    if(largest<1e-12)break;
    const double tau=(n[q][q]-n[p][p])/(2*n[p][q]);
    const double t=std::copysign(1.0,tau)/(std::abs(tau)+std::sqrt(1+tau*tau));
    const double c=1/std::sqrt(1+t*t),s=t*c;
    const double app=n[p][p],aqq=n[q][q],apq=n[p][q];
    n[p][p]=app-t*apq;n[q][q]=aqq+t*apq;n[p][q]=n[q][p]=0;
    for(int k=0;k<4;++k) {
      if(k!=p&&k!=q) {
        const double akp=n[k][p],akq=n[k][q];
        n[k][p]=n[p][k]=c*akp-s*akq;
        n[k][q]=n[q][k]=s*akp+c*akq;
      }
      const double vkp=vectors[k][p],vkq=vectors[k][q];
      vectors[k][p]=c*vkp-s*vkq;vectors[k][q]=s*vkp+c*vkq;
    }
  }
  int bestEigen=0;
  for(int i=1;i<4;++i)if(n[i][i]>n[bestEigen][bestEigen])bestEigen=i;
  const QQuaternion rotation{float(vectors[0][bestEigen]),float(vectors[1][bestEigen]),
                             float(vectors[2][bestEigen]),float(vectors[3][bestEigen])};
  if(rotation.isNull())return false;
  const QQuaternion unit=rotation.normalized();
  for(int col=0;col<3;++col) {
    QVector3D axis(col==0,col==1,col==2);axis=unit.rotatedVector(axis);
    for(int row=0;row<3;++row)pose.m[row*4+col]=axis[row];
  }
  const QVector3D translation=qc-unit.rotatedVector(pc);
  pose.m[3]=translation.x();pose.m[7]=translation.y();pose.m[11]=translation.z();
  return true;
}
}

ScannerVisualResult ScannerVisualTracker::align(const QByteArray& currentDepth,
    const QByteArray& referenceDepth,const DepthIntrinsics& camera,const QImage& currentColor,
    const QImage& referenceColor) const {
  ScannerVisualResult result;
  const Grid current=registered(currentDepth,camera,currentColor);
  const Grid reference=registered(referenceDepth,camera,referenceColor);
  const auto pairs=match(current,reference);
  result.matches=int(pairs.size());
  if(pairs.size()<8)return result;
  std::mt19937 random(0x5ca7u);
  std::uniform_int_distribution<int> pick(0,int(pairs.size())-1);
  std::vector<int> best;
  for(int trial=0;trial<100;++trial) {
    std::vector<int> seed;
    while(seed.size()<3) {
      const int next=pick(random);
      if(std::find(seed.begin(),seed.end(),next)==seed.end())seed.push_back(next);
    }
    ScannerPose candidate;
    if(!rigid(pairs,seed,candidate))continue;
    std::vector<int> inliers;
    for(int i=0;i<int(pairs.size());++i)
      if((candidate.world(pairs[size_t(i)].current)-pairs[size_t(i)].reference).length()<.035f)
        inliers.push_back(i);
    if(inliers.size()>best.size())best=std::move(inliers);
  }
  result.inliers=int(best.size());
  if(best.size()<8||best.size()*3<pairs.size()||!rigid(pairs,best,result.currentToReference))return result;
  double squared=0;
  for(int index:best) {
    const float error=(result.currentToReference.world(pairs[size_t(index)].current)-
        pairs[size_t(index)].reference).length();
    squared+=error*error;
  }
  result.rmsM=float(std::sqrt(squared/best.size()));
  const QVector3D translation(result.currentToReference.m[3],result.currentToReference.m[7],
                              result.currentToReference.m[11]);
  const float trace=result.currentToReference.m[0]+result.currentToReference.m[5]+
                    result.currentToReference.m[10];
  const float angle=std::acos(std::clamp((trace-1)*.5f,-1.f,1.f));
  result.accepted=result.rmsM<.025f&&translation.length()<.20f&&angle<.35f;
  return result;
}
