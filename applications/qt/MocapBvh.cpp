#include "MocapBvh.hpp"

#include <algorithm>
#include <cmath>

namespace {
QByteArray number(double n) {return QByteArray::number(n,'f',4);}
QByteArray offset(const QVector3D& u) {
  return number(-u.x()*100)+" "+number(u.y()*100)+" "+number(u.z()*100);
}
}

bool MocapBvh::start(const QString& path) {
  if(active_) stop();
  frames_=0;frameCountOffset_=frameTimeOffset_=0;first_=last_=0;error_.clear();
  file_.setFileName(path);
  if(!file_.open(QIODevice::WriteOnly)) {error_=file_.errorString();return false;}
  active_=true;return true;
}

QVector3D MocapBvh::eulerZxy(QQuaternion q) {
  q=QQuaternion(q.scalar(),q.x(),-q.y(),-q.z()).normalized();
  const QMatrix3x3 m=q.toRotationMatrix();
  const double sx=std::clamp(double(m(2,1)),-1.,1.);
  const double x=std::asin(sx);
  const double z=std::abs(sx)<.99999?std::atan2(-m(0,1),m(1,1)):
      std::atan2(m(1,0),m(0,0));
  const double y=std::abs(sx)<.99999?std::atan2(-m(2,0),m(2,2)):0;
  constexpr double deg=180.0/3.14159265358979323846;
  return {float(z*deg),float(x*deg),float(y*deg)};
}

void MocapBvh::append(const HumanoidPose& pose) {
  if(!active_||!pose.tracked) return;
  if(frames_==0) {
    for(int i=1;i<HumanoidPose::count;++i)offsets_[i]=pose.bones[i].localPosition;
    QByteArray header="HIERARCHY\n";joint(header,0,0);
    header+="MOTION\nFrames: ";frameCountOffset_=header.size();
    header+=QByteArray(20,'0');header+="\nFrame Time: ";
    frameTimeOffset_=header.size();header+=QByteArray(18,' ');header+='\n';
    if(file_.write(header)!=header.size()) {
      error_=file_.errorString();file_.cancelWriting();active_=false;return;
    }
  }
  QByteArray row;const QVector3D root=pose.bones[0].position;
  row+=offset(root);
  for(const HumanoidBone& bone:pose.bones) {
    const QVector3D angles=eulerZxy(bone.localRotation);
    row+=" "+number(angles.x())+" "+number(angles.y())+" "+number(angles.z());
  }
  row+='\n';
  if(file_.write(row)!=row.size()) {
    error_=file_.errorString();file_.cancelWriting();active_=false;return;
  }
  ++frames_;
  if(first_==0) first_=pose.time;last_=pose.time;
}

void MocapBvh::joint(QByteArray& out,int bone,int depth) const {
  const QByteArray indent(depth*2,' ');
  if(bone==0) {
    out+="ROOT Hips\n{\n  OFFSET 0.0000 0.0000 0.0000\n";
    out+="  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n";
  } else {
    out+=indent+"JOINT "+MocapRig::name(bone)+"\n"+indent+"{\n";
    out+=indent+"  OFFSET "+offset(offsets_[bone])+"\n";
    out+=indent+"  CHANNELS 3 Zrotation Xrotation Yrotation\n";
  }
  bool hasChild=false;
  for(int child=1;child<HumanoidPose::count;++child)
    if(MocapRig::parent(child)==bone) {joint(out,child,depth+1);hasChild=true;}
  if(!hasChild) {
    QVector3D tip{0,.18f,0};
    if(bone==8) tip={-.16f,0,0};
    if(bone==12) tip={.16f,0,0};
    if(bone==15||bone==18) tip={0,-.05f,.12f};
    out+=indent+"  End Site\n"+indent+"  {\n"+indent+"    OFFSET "+
         offset(tip)+"\n"+indent+"  }\n";
  }
  out+=indent+"}\n";
}

bool MocapBvh::stop() {
  if(!active_) return error_.isEmpty();
  active_=false;
  if(frames_==0) {file_.cancelWriting();return true;}
  const double frameTime=frames_>1&&last_>first_?
      (last_-first_)/double(frames_-1):1.0/30.0;
  const QByteArray count=QByteArray::number(frames_).rightJustified(20,'0');
  const QByteArray timing=QByteArray::number(frameTime,'f',9).rightJustified(18,' ');
  if(!file_.seek(frameCountOffset_)||file_.write(count)!=count.size()||
     !file_.seek(frameTimeOffset_)||file_.write(timing)!=timing.size()) {
    error_=file_.errorString();file_.cancelWriting();return false;
  }
  if(!file_.commit()) {error_=file_.errorString();return false;}
  return true;
}
