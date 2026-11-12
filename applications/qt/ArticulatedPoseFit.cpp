#include "ArticulatedPoseFit.hpp"

#include <QJsonArray>
#include <algorithm>
#include <cmath>

namespace {
constexpr const char* names[]{"shoulderL","shoulderR","elbowL","elbowR",
    "wristL","wristR","hipL","hipR","kneeL","kneeR","ankleL","ankleR"};
QVector3D point(const QJsonArray& p) {
  return {float(p[0].toDouble()),float(p[1].toDouble()),float(p[2].toDouble())};
}
}

void ArticulatedPoseFit::reset() {
  for(auto& bone:bones_) {bone.samples.clear();bone.length=0;bone.locked=false;}
  previousValid_.fill(false);lastTime_=0;frames_=0;device_.clear();
}

QJsonObject ArticulatedPoseFit::fit(QJsonObject frame) {
  const QString device=frame.value("device").toString();
  if(device!=device_) {reset();device_=device;}
  const double time=frame.value("t").toDouble();
  if(lastTime_>0&&(time<=lastTime_||time-lastTime_>.5)) previousValid_.fill(false);
  QJsonArray bodies=frame.value("bodies").toArray();
  if(bodies.isEmpty()) {previousValid_.fill(false);lastTime_=time;return frame;}
  ++frames_;
  QJsonObject body=bodies[0].toObject(), json=body.value("joints").toObject();
  std::array<Joint,count> joints{};
  for(int i=0;i<count;++i) {
    const QJsonObject j=json.value(names[i]).toObject();
    const QJsonArray p=j.value("p").toArray();
    if(p.size()!=3)continue;
    const QVector3D v=point(p);
    const float c=float(j.value("c").toDouble());
    if(!std::isfinite(v.x())||!std::isfinite(v.y())||!std::isfinite(v.z())||
        v.z()<.35f||c<.15f)continue;
    joints[i]={v,v,c,true};
  }
  // Learn lengths only from clear, plausible observations. A median resists an
  // occasional depth sample on the wall or the front of a limb.
  for(auto& bone:bones_) {
    if(bone.locked)continue;
    const Joint& a=joints[bone.a],& b=joints[bone.b];
    if(a.valid&&b.valid&&a.confidence>.35f&&b.confidence>.35f) {
      const float d=(a.observed-b.observed).length();
      if(d>=bone.low&&d<=bone.high&&bone.samples.size()<30)
        bone.samples.push_back(d);
    }
    if(bone.samples.size()>=12) {
      auto sorted=bone.samples;
      std::sort(sorted.begin(),sorted.end());
      bone.length=sorted[sorted.size()/2];
      if(bone.samples.size()==30)bone.locked=true;
    }
  }
  // Distal joints are often too weak for direct length calibration. Scale their
  // anatomical proportions from a measured proximal segment after the initial
  // calibration window, and freeze that fallback for this subject.
  if(frames_>=45)for(int i=0;i<int(bones_.size());++i) {
    Bone& bone=bones_[i];
    if(bone.locked)continue;
    if(bone.length<=0) {
      const int reference=i==5?4:i==7?6:i==9?8:i==11?10:-1;
      const float scale=reference>=0&&bones_[reference].length>0?
          bones_[reference].length/bones_[reference].nominal:1.f;
      bone.length=std::clamp(bone.nominal*scale,bone.low,bone.high);
    }
    bone.locked=true;
  }
  // Fit all connected joints together. Observation and temporal terms compete
  // with fixed bone lengths; each projection moves both ends according to their
  // confidence, so a clear joint anchors an uncertain neighbour.
  for(int i=0;i<count;++i)if(!joints[i].valid&&previousValid_[i]&&
      lastTime_>0&&time-lastTime_>0&&time-lastTime_<.12) {
    joints[i].position=previous_[i];joints[i].confidence=.06f;joints[i].valid=true;
  }
  for(int iteration=0;iteration<24;++iteration) {
    for(auto& j:joints)if(j.valid&&j.confidence>=.15f)
      j.position+=(j.observed-j.position)*(iteration==0?.30f:.12f)*
          std::clamp(j.confidence,.2f,1.f);
    for(const auto& bone:bones_) {
      if(bone.length<=0)continue;
      Joint& a=joints[bone.a];Joint& b=joints[bone.b];
      if(!a.valid||!b.valid)continue;
      QVector3D delta=b.position-a.position;
      const float distance=delta.length();
      if(distance<.03f)continue;
      const QVector3D correction=delta*((distance-bone.length)/distance*.75f);
      const float wa=1.f/std::max(.08f,a.confidence);
      const float wb=1.f/std::max(.08f,b.confidence);
      a.position+=correction*(wa/(wa+wb));
      b.position-=correction*(wb/(wa+wb));
    }
  }
  // The limb graph is a tree below the shoulders and hips. A final outward
  // pass makes every calibrated arm and leg segment exactly its fixed length.
  for(int index:{4,5,6,7,8,9,10,11}) {
    const Bone& bone=bones_[index];
    if(bone.length<=0)continue;
    Joint& a=joints[bone.a];Joint& b=joints[bone.b];
    if(!a.valid||!b.valid)continue;
    const QVector3D delta=b.position-a.position;
    if(delta.length()>.03f)b.position=a.position+delta.normalized()*bone.length;
  }
  for(int i=0;i<count;++i) {
    previousValid_[i]=joints[i].valid;
    if(!joints[i].valid) {json.remove(names[i]);continue;}
    previous_[i]=joints[i].position;
    // Predicted occluded joints retain their last image location and low
    // confidence; they are not counted as a fresh observation.
    QJsonObject j=json.value(names[i]).toObject();
    if(j.isEmpty())continue;
    const auto& p=joints[i].position;
    j["p"]=QJsonArray{p.x(),p.y(),p.z()};json[names[i]]=j;
  }
  lastTime_=time;
  body["joints"]=json;bodies[0]=body;frame["bodies"]=bodies;
  return frame;
}
