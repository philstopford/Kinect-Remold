// Kinect v2 factory polynomial mapping is adapted from OpenKinect libfreenect2
// RegistrationImpl::depth_to_color (Apache-2.0 option). Copyright (c) 2014
// individual OpenKinect contributors. See third_party/licenses/libfreenect2.
#include "RgbdRegistration.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {
quint16 mmAt(const QByteArray& data,int index) {
  const auto* p=reinterpret_cast<const quint8*>(data.constData()+index*2);
  return quint16(p[0]|(quint16(p[1])<<8));
}
float field(const QByteArray& data,int offset) {
  float value=0;
  if(offset>=0&&offset+4<=data.size()) std::memcpy(&value,data.constData()+offset,4);
  return value;
}
struct ProjectedPoint {float u,v;QVector3D xyz;};
}

bool RgbdRegistration::projectKinectOne(const DepthIntrinsics& camera,int x,int y,
                                        quint16 millimeters,int colorWidth,int colorHeight,
                                        float& u,float& v) {
  if(camera.extension.size()<144||!(quint8(camera.extension[12])&4u)||
     millimeters<1||colorWidth<1||colorHeight<1)return false;
  const float fx=field(camera.extension,40),cx=field(camera.extension,48),
              cy=field(camera.extension,52),shiftD=field(camera.extension,56),
              shiftM=field(camera.extension,60);
  if(!(fx>100&&shiftD>1&&std::isfinite(shiftM)))return false;
  double rayX=0,rayY=0;
  if(!camera.ray(x,y,rayX,rayY))return false;
  const float mx=float(rayX*camera.fx)*.01f;
  const float my=float(rayY*camera.fy)*.01f;
  const float powers[]{mx*mx*mx,my*my*my,mx*mx*my,my*my*mx,
      mx*mx,my*my,mx*my,mx,my,1.f};
  float wx=0,wy=0;
  for(int i=0;i<10;++i) {
    wx+=powers[i]*field(camera.extension,64+i*4);
    wy+=powers[i]*field(camera.extension,104+i*4);
  }
  constexpr float colorQ=.002199f;
  const float rx=wx/(fx*colorQ)-shiftM/shiftD+shiftM/millimeters;
  const float fullU=rx*fx+cx,fullV=wy/colorQ+cy;
  if(!std::isfinite(fullU)||!std::isfinite(fullV))return false;
  u=fullU*colorWidth/1920.f;v=fullV*colorHeight/1080.f;
  return true;
}

std::array<MetricPoseJoint,RgbdRegistration::jointCount> RgbdRegistration::lift(
    const QByteArray& depth,const DepthIntrinsics& camera,
    const DepthCalibrationProfile* calibration,int colorWidth,int colorHeight,
    bool kinectOne,const std::array<BodyKeypoint,17>& body,
    const std::array<std::array<BodyKeypoint,21>,2>& hands,
    float expectedPersonDepth) {
  std::array<MetricPoseJoint,jointCount> result{};
  if(!camera.valid()||depth.size()!=qsizetype(camera.width)*camera.height*2||
     colorWidth<1||colorHeight<1) return result;
  const bool factoryColor=kinectOne&&camera.extension.size()>=64&&
      (quint8(camera.extension[12])&2u)&&field(camera.extension,40)>100;
  const bool factoryPolynomial=factoryColor&&camera.extension.size()>=144&&
      (quint8(camera.extension[12])&4u);
  const float fx=factoryColor?field(camera.extension,40)*colorWidth/1920.0f:
      529.215081f*colorWidth/640.0f;
  const float fy=factoryColor?field(camera.extension,44)*colorHeight/1080.0f:
      525.563936f*colorHeight/480.0f;
  const float cx=factoryColor?field(camera.extension,48)*colorWidth/1920.0f:
      328.942720f*colorWidth/640.0f;
  const float cy=factoryColor?field(camera.extension,52)*colorHeight/1080.0f:
      267.480682f*colorHeight/480.0f;
  if(kinectOne&&!factoryColor) return result;
  std::vector<ProjectedPoint> projected;
  projected.reserve(size_t(camera.width/2)*camera.height/2);
  for(int y=1;y<camera.height-1;y+=2) for(int x=1;x<camera.width-1;x+=2) {
    quint16 mm=mmAt(depth,y*camera.width+x);
    if(calibration) mm=calibration->correct(x,y,mm);
    if(mm<350||mm>6000) continue;
    double rx=0,ry=0;
    if(!camera.ray(x,y,rx,ry)) continue;
    const float z=mm*.001f;
    const QVector3D p(float(rx)*z,float(ry)*z,z);
    float xc=p.x(),yc=p.y(),zc=p.z();
    if(!kinectOne) {
      xc=.9998463f*p.x()+.0012635f*p.y()-.0174872f*p.z()+.01998524f;
      yc=-.0014779f*p.x()+.9999239f*p.y()-.0122514f*p.z()-.00074424f;
      zc=.0174704f*p.x()+.0122753f*p.y()+.9997720f*p.z()-.01091674f;
    }
    if(zc<=.05f) continue;
    float nx=xc/zc,ny=yc/zc;
    if(!kinectOne) {
      const float rr=nx*nx+ny*ny,radial=1+.207966f*rr-.586138f*rr*rr+.498570f*rr*rr*rr;
      const float dx=nx*radial+2*.000722f*nx*ny+.001048f*(rr+2*nx*nx);
      const float dy=ny*radial+.000722f*(rr+2*ny*ny)+2*.001048f*nx*ny;
      nx=dx;ny=dy;
    }
    float u=fx*nx+cx,v=fy*ny+cy;
    if(factoryPolynomial&&
       !projectKinectOne(camera,x,y,mm,colorWidth,colorHeight,u,v))continue;
    if(u<0||v<0||u>=colorWidth||v>=colorHeight) continue;
    projected.push_back({u,v,p});
  }
  std::array<BodyKeypoint,jointCount> imagePoints{};
  const int mapped[16]{0,0,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  for(int i=0;i<16;++i) imagePoints[i]=body[mapped[i]];
  for(int hand=0;hand<2;++hand) for(int i=0;i<21;++i)
    imagePoints[16+hand*21+i]=hands[hand][i];
  const float maxDistance=std::max(12.0f,colorWidth/1920.0f*34.0f);
  // Index projected depth by color pixel. A joint only needs candidates inside
  // maxDistance, so scanning the entire depth cloud for every joint is wasteful.
  constexpr int binSize=16;
  const int binCols=(colorWidth+binSize-1)/binSize;
  const int binRows=(colorHeight+binSize-1)/binSize;
  std::vector<std::vector<int>> bins(size_t(binCols)*binRows);
  for(int index=0;index<int(projected.size());++index) {
    const auto& p=projected[size_t(index)];
    bins[size_t(int(p.v)/binSize)*binCols+int(p.u)/binSize].push_back(index);
  }
  auto nearby=[&](float u,float v,const auto& visit) {
    const int left=std::max(0,int(std::floor((u-maxDistance)/binSize)));
    const int right=std::min(binCols-1,int(std::floor((u+maxDistance)/binSize)));
    const int top=std::max(0,int(std::floor((v-maxDistance)/binSize)));
    const int bottom=std::min(binRows-1,int(std::floor((v+maxDistance)/binSize)));
    for(int by=top;by<=bottom;++by)for(int bx=left;bx<=right;++bx)
      for(int index:bins[size_t(by)*binCols+bx])visit(projected[size_t(index)]);
  };
  for(int i=0;i<jointCount;++i) {
    const BodyKeypoint keypoint=imagePoints[i];
    if(keypoint.confidence<(i>=16?.45f:.14f)||!std::isfinite(keypoint.x)||
       !std::isfinite(keypoint.y)||keypoint.x<0||keypoint.x>1||
       keypoint.y<0||keypoint.y>1) continue;
    const float targetU=keypoint.x*colorWidth,targetV=keypoint.y*colorHeight;
    float best=maxDistance*maxDistance;
    const ProjectedPoint* selected=nullptr;
    nearby(targetU,targetV,[&](const ProjectedPoint& p) {
      if(expectedPersonDepth>0&&
         std::abs(p.xyz.z()-expectedPersonDepth)>(i>=16?.9f:i>=12?.35f:i>=10?.55f:.7f))return;
      const float du=p.u-targetU,dv=p.v-targetV,distance=du*du+dv*dv;
      if(distance<best) {best=distance;selected=&p;}
    });
    if(!selected) continue;
    const float registrationQuality=std::max(.1f,1-std::sqrt(best)/maxDistance);
    result[i]={selected->xyz,keypoint.confidence*registrationQuality,true};
  }
  // Nearest color projection can land on the background at a silhouette edge.
  // Use the central torso to restrict limb depth, then reject impossible bone
  // lengths. Invalid joints are preferable to a meter-long jumping limb.
  std::vector<float> torsoDepth;
  for(int i:{4,5,10,11})if(result[i].valid)torsoDepth.push_back(result[i].point.z());
  if(torsoDepth.size()>=2) {
    std::sort(torsoDepth.begin(),torsoDepth.end());
    const float torsoZ=torsoDepth[torsoDepth.size()/2];
    for(int i:{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}) {
      const BodyKeypoint keypoint=imagePoints[i];
      if(keypoint.confidence<.14f)continue;
      const float depthRange=i==4||i==5||i==10||i==11?.6f:
          i<=3?.9f:i<=9?.75f:1.2f;
      const float targetU=keypoint.x*colorWidth,targetV=keypoint.y*colorHeight;
      float best=maxDistance*maxDistance;
      float bestDistance=best;
      const ProjectedPoint* selected=nullptr;
      nearby(targetU,targetV,[&](const ProjectedPoint& p) {
        if(expectedPersonDepth>0&&
           std::abs(p.xyz.z()-expectedPersonDepth)>(i>=12?.35f:i>=10?.55f:.7f))return;
        if(std::abs(p.xyz.z()-torsoZ)>depthRange)return;
        if(i>=12&&std::abs(p.xyz.z()-torsoZ)>.35f)return;
        const float du=p.u-targetU,dv=p.v-targetV,distance=du*du+dv*dv;
        const float score=distance+(i>=12?2500.f*(p.xyz.z()-torsoZ)*
            (p.xyz.z()-torsoZ):0.f);
        if(score<best) {best=score;bestDistance=distance;selected=&p;}
      });
      if(!selected) {result[i]={};continue;}
      const float quality=std::max(.1f,1-std::sqrt(bestDistance)/maxDistance);
      result[i]={selected->xyz,keypoint.confidence*quality,true};
    }
    const int parents[16]{-1,-1,-1,-1,-1,-1,4,5,6,7,-1,-1,10,11,12,13};
    for(int i:{6,7,8,9,12,13,14,15}) {
      const int parent=parents[i];
      if(!result[i].valid||!result[parent].valid)continue;
      const float maxLength=i<=9?.7f:.95f;
      if((result[i].point-result[parent].point).length()>maxLength)result[i]={};
    }
  }
  return result;
}
