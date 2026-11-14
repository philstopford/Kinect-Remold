#include "ScannerIcp.hpp"

#include <algorithm>
#include <cmath>

namespace {
quint16 mmAt(const QByteArray& data,int index) {
  const auto* p=reinterpret_cast<const quint8*>(data.constData()+index*2);
  return quint16(p[0]|(quint16(p[1])<<8));
}
QVector3D pointAt(const QByteArray& data,const DepthIntrinsics& camera,
                  const DepthCalibrationProfile* calibration,int x,int y) {
  if(x<0||y<0||x>=camera.width||y>=camera.height) return {};
  quint16 mm=mmAt(data,y*camera.width+x);
  if(calibration) mm=calibration->correct(x,y,mm);
  if(mm<350||mm>6000) return {};
  double rx=0,ry=0;
  if(!camera.ray(x,y,rx,ry)) return {};
  const float z=mm*.001f;
  return {float(rx)*z,float(ry)*z,z};
}
bool solve6(double A[6][7],float delta[6]) {
  for(int col=0;col<6;++col) {
    int pivot=col;
    for(int row=col+1;row<6;++row)
      if(std::abs(A[row][col])>std::abs(A[pivot][col])) pivot=row;
    if(std::abs(A[pivot][col])<1e-10) return false;
    for(int k=col;k<7;++k) std::swap(A[pivot][k],A[col][k]);
    const double divisor=A[col][col];
    for(int k=col;k<7;++k) A[col][k]/=divisor;
    for(int row=0;row<6;++row) if(row!=col) {
      const double scale=A[row][col];
      for(int k=col;k<7;++k) A[row][k]-=scale*A[col][k];
    }
  }
  for(int i=0;i<6;++i) {
    if(!std::isfinite(A[i][6])) return false;
    delta[i]=float(A[i][6]);
  }
  return true;
}
ScannerPose increment(const float d[6]) {
  ScannerPose pose;
  const float wx=d[0],wy=d[1],wz=d[2];
  const float theta=std::sqrt(wx*wx+wy*wy+wz*wz);
  const float A=theta<1e-5f?1-theta*theta/6:std::sin(theta)/theta;
  const float B=theta<1e-5f?.5f-theta*theta/24:(1-std::cos(theta))/(theta*theta);
  const float w[3][3]{{0,-wz,wy},{wz,0,-wx},{-wy,wx,0}};
  for(int row=0;row<3;++row) for(int col=0;col<3;++col) {
    float square=0;
    for(int k=0;k<3;++k) square+=w[row][k]*w[k][col];
    pose.m[row*4+col]=(row==col?1.0f:0.0f)+A*w[row][col]+B*square;
  }
  pose.m[3]=d[3];pose.m[7]=d[4];pose.m[11]=d[5];
  return pose;
}
}

ScannerIcpResult ScannerIcp::align(const QByteArray& current,const QByteArray& reference,
    const DepthIntrinsics& camera,const DepthCalibrationProfile* calibration,bool turntable,
    const ScannerPose* initial) const {
  ScannerIcpResult result;
  if(!camera.valid()||current.size()!=qsizetype(camera.width)*camera.height*2||
     reference.size()!=current.size()) return result;
  ScannerPose estimate=initial?*initial:ScannerPose{};
  int finalMatches=0,finalSamples=0;
  float finalRms=1;
  float finalObservability=0;
  for(int level=0;level<3;++level) {
    const int stride=level==0?8:level==1?5:3;
    const float gate=level==0?.09f:level==1?.055f:.035f;
    for(int iteration=0;iteration<6;++iteration) {
      double A[6][7]{};double squared=0,weightSum=0;
      int matches=0,samples=0;
      for(int y=stride;y<camera.height-stride;y+=stride)
        for(int x=stride;x<camera.width-stride;x+=stride) {
          const QVector3D source=pointAt(current,camera,calibration,x,y);
          if(source.z()<=0) continue;
          ++samples;
          const QVector3D p=estimate.world(source);
          if(p.z()<=0) continue;
          const int u=int(std::lround(camera.fx*p.x()/p.z()+camera.cx));
          const int v=int(std::lround(camera.fy*p.y()/p.z()+camera.cy));
          if(u<2||v<2||u>=camera.width-2||v>=camera.height-2) continue;
          const QVector3D q=pointAt(reference,camera,calibration,u,v);
          const QVector3D l=pointAt(reference,camera,calibration,u-1,v);
          const QVector3D r=pointAt(reference,camera,calibration,u+1,v);
          const QVector3D t=pointAt(reference,camera,calibration,u,v-1);
          const QVector3D b=pointAt(reference,camera,calibration,u,v+1);
          if(q.z()<=0||l.z()<=0||r.z()<=0||t.z()<=0||b.z()<=0||
             std::abs(l.z()-r.z())>gate||std::abs(t.z()-b.z())>gate||
             std::abs(p.z()-q.z())>gate) continue;
          QVector3D normal=QVector3D::crossProduct(r-l,b-t);
          if(normal.lengthSquared()<1e-10f) continue;
          normal.normalize();
          const float residual=QVector3D::dotProduct(p-q,normal);
          if(std::abs(residual)>gate) continue;
          const float huber=.015f;
          const double weight=std::abs(residual)<=huber?1.0:huber/std::abs(residual);
          const QVector3D cross=QVector3D::crossProduct(p,normal);
          const double J[6]{cross.x(),cross.y(),cross.z(),normal.x(),normal.y(),normal.z()};
          for(int row=0;row<6;++row) {
            A[row][6]-=weight*J[row]*residual;
            for(int col=0;col<6;++col) A[row][col]+=weight*J[row]*J[col];
          }
          squared+=weight*residual*residual;weightSum+=weight;++matches;
        }
      if(matches<100||samples<200||weightSum<=0) break;
      finalMatches=matches;finalSamples=samples;finalRms=float(std::sqrt(squared/weightSum));
      const double translationSum=A[3][3]+A[4][4]+A[5][5];
      finalObservability=translationSum>0?float(std::min({A[3][3],A[4][4],A[5][5]})/
          translationSum):0;
      if(turntable) for(int fixed:{0,2,4}) {
        for(int k=0;k<6;++k) A[fixed][k]=A[k][fixed]=0;
        A[fixed][fixed]=1;A[fixed][6]=0;
      }
      for(int k=0;k<6;++k) A[k][k]+=1e-5;
      float delta[6]{};
      if(!solve6(A,delta)) break;
      const float rotation=std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
      const float translation=std::sqrt(delta[3]*delta[3]+delta[4]*delta[4]+delta[5]*delta[5]);
      if(rotation>.14f||translation>(turntable?.12f:.05f)) break;
      estimate=increment(delta).compose(estimate);
      if(rotation<.00005f&&translation<.00005f) break;
    }
  }
  result.currentToReference=estimate;
  result.matches=finalMatches;result.overlap=finalSamples?float(finalMatches)/finalSamples:0;
  result.rmsM=finalRms;
  result.translationObservability=finalObservability;
  const QVector3D translation(estimate.m[3],estimate.m[7],estimate.m[11]);
  const float trace=estimate.m[0]+estimate.m[5]+estimate.m[10];
  const float angle=std::acos(std::clamp((trace-1)*.5f,-1.f,1.f));
  result.accepted=finalMatches>=120&&result.overlap>=.18f&&finalRms<.025f&&
      translation.length()<.20f&&angle<.35f;
  return result;
}
