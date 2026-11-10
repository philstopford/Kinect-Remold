#include "ScannerSegmentation.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace {
quint16 mmAt(const QByteArray& data,int index) {
  const auto* p=reinterpret_cast<const quint8*>(data.constData()+index*2);
  return quint16(p[0]|(quint16(p[1])<<8));
}
QVector3D point(int x,int y,quint16 mm,const DepthIntrinsics& camera) {
  double rx=0,ry=0;
  if(!camera.ray(x,y,rx,ry)) return {};
  const float z=mm*.001f;return {float(rx)*z,float(ry)*z,z};
}
}

ScannerSupportPlane ScannerSegmentation::findSupport(const QByteArray& depth,
    const DepthIntrinsics& camera,const DepthCalibrationProfile* calibration) {
  ScannerSupportPlane result;
  if(!camera.valid()||depth.size()!=qsizetype(camera.width)*camera.height*2) return result;
  std::vector<QVector3D> samples;
  for(int y=camera.height/3;y<camera.height-4;y+=4)
    for(int x=4;x<camera.width-4;x+=4) {
      quint16 mm=mmAt(depth,y*camera.width+x);
      if(calibration) mm=calibration->correct(x,y,mm);
      if(mm<350||mm>4000) continue;
      const QVector3D p=point(x,y,mm,camera);
      if(p.z()>0) samples.push_back(p);
    }
  if(samples.size()<250) return result;
  std::mt19937 random(0x5ca7u);
  std::uniform_int_distribution<size_t> pick(0,samples.size()-1);
  int best=0;
  QVector3D bestNormal;float bestOffset=0;
  for(int trial=0;trial<140;++trial) {
    const QVector3D a=samples[pick(random)],b=samples[pick(random)],c=samples[pick(random)];
    QVector3D n=QVector3D::crossProduct(b-a,c-a);
    if(n.lengthSquared()<1e-6f) continue;
    n.normalize();if(n.y()<0) n=-n;
    if(n.y()<.72f) continue;
    const float d=-QVector3D::dotProduct(n,a);
    int inliers=0;
    for(const QVector3D& p:samples)
      if(std::abs(QVector3D::dotProduct(n,p)+d)<.018f) ++inliers;
    if(inliers>best) {best=inliers;bestNormal=n;bestOffset=d;}
  }
  result.inliers=best;result.coverage=float(best)/samples.size();
  result.valid=best>=200&&result.coverage>=.18f;
  if(result.valid) {result.normal=bestNormal;result.offset=bestOffset;}
  return result;
}

void ScannerSupportPlane::removeSupport(QByteArray& depth,const DepthIntrinsics& camera,
    const DepthCalibrationProfile* calibration) const {
  if(!valid||!camera.valid()||depth.size()!=qsizetype(camera.width)*camera.height*2) return;
  for(int y=0;y<camera.height;++y) for(int x=0;x<camera.width;++x) {
    const int index=y*camera.width+x;
    quint16 mm=mmAt(depth,index);
    if(mm<350||mm>6000) continue;
    if(calibration) mm=calibration->correct(x,y,mm);
    const QVector3D p=point(x,y,mm,camera);
    if(QVector3D::dotProduct(normal,p)+offset>=-.025f) {
      depth[index*2]=0;depth[index*2+1]=0;
    }
  }
}
