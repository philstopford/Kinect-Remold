#include "MocapRig.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QMatrix3x3>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr const char* names[]{"Hips","Spine","Chest","Neck","Head",
    "LeftShoulder","LeftUpperArm","LeftLowerArm","LeftHand",
    "RightShoulder","RightUpperArm","RightLowerArm","RightHand",
    "LeftUpperLeg","LeftLowerLeg","LeftFoot","RightUpperLeg","RightLowerLeg","RightFoot"};
constexpr int parents[]{-1,0,1,2,3,2,5,6,7,2,9,10,11,0,13,14,0,16,17};
constexpr int mirrors[]{0,1,2,3,4,9,10,11,12,5,6,7,8,16,17,18,13,14,15};
QVector3D point(const QJsonObject& joints,const char* name,bool& valid,float& confidence) {
  const QJsonObject joint=joints.value(QLatin1String(name)).toObject();
  const QJsonArray p=joint.value("p").toArray();
  confidence=float(joint.value("c").toDouble());valid=p.size()==3&&confidence>0;
  if(!valid) return {};
  return {-float(p[0].toDouble()),-float(p[1].toDouble()),-float(p[2].toDouble())};
}
QVector3D midpoint(const QVector3D& a,const QVector3D& b) {return (a+b)*.5f;}
QQuaternion swing(QVector3D from,QVector3D to) {
  if(from.lengthSquared()<1e-8f||to.lengthSquared()<1e-8f) return {};
  return QQuaternion::rotationTo(from.normalized(),to.normalized()).normalized();
}
QQuaternion torsoFrame(QVector3D up,QVector3D side) {
  if(up.lengthSquared()<1e-8f||side.lengthSquared()<1e-8f) return {};
  up.normalize();
  side-=up*QVector3D::dotProduct(side,up);
  if(side.lengthSquared()<1e-6f) return {};
  side.normalize();
  const QVector3D forward=QVector3D::crossProduct(side,up).normalized();
  side=QVector3D::crossProduct(up,forward).normalized();
  QMatrix3x3 m;
  m(0,0)=side.x();m(1,0)=side.y();m(2,0)=side.z();
  m(0,1)=up.x();m(1,1)=up.y();m(2,1)=up.z();
  m(0,2)=forward.x();m(1,2)=forward.y();m(2,2)=forward.z();
  return QQuaternion::fromRotationMatrix(m).normalized();
}
QQuaternion frameRotation(QVector3D restPrimary,QVector3D restSide,
                          QVector3D primary,QVector3D side) {
  if(restPrimary.lengthSquared()<1e-8f||primary.lengthSquared()<1e-8f) return {};
  restPrimary.normalize();primary.normalize();
  restSide-=restPrimary*QVector3D::dotProduct(restSide,restPrimary);
  side-=primary*QVector3D::dotProduct(side,primary);
  if(restSide.lengthSquared()<1e-6f||side.lengthSquared()<1e-6f) return {};
  restSide.normalize();side.normalize();
  const QVector3D restThird=QVector3D::crossProduct(restPrimary,restSide);
  const QVector3D third=QVector3D::crossProduct(primary,side);
  QMatrix3x3 m;
  const QVector3D a[]{restPrimary,restSide,restThird};
  const QVector3D b[]{primary,side,third};
  for(int row=0;row<3;++row) for(int col=0;col<3;++col)
    m(row,col)=b[0][row]*a[0][col]+b[1][row]*a[1][col]+b[2][row]*a[2][col];
  return QQuaternion::fromRotationMatrix(m).normalized();
}
QQuaternion parentSwing(const QQuaternion& parent,QVector3D rest,QVector3D observed) {
  return (swing(parent.rotatedVector(rest),observed)*parent).normalized();
}
float smoothstep(float low,float high,float value) {
  const float t=std::clamp((value-low)/(high-low),0.f,1.f);
  return t*t*(3.f-2.f*t);
}
QJsonArray vec(const QVector3D& v) {return {v.x(),v.y(),v.z()};}
QJsonArray quat(const QQuaternion& q) {return {q.x(),q.y(),q.z(),q.scalar()};}
}

const char* MocapRig::name(int bone) {return names[bone];}
int MocapRig::parent(int bone) {return parents[bone];}
QVector3D MocapRig::restOffset(int i) {
  static const QVector3D offsets[]{
    {0,0,0},{0,.21f,0},{0,.14f,0},{0,.14f,0},{0,.19f,0},
    {-.046f,.14f,0},{-.139f,0,0},{-.29f,0,0},{-.255f,0,0},
    {.046f,.14f,0},{.139f,0,0},{.29f,0,0},{.255f,0,0},
    {-.0475f,0,0},{0,-.44f,0},{0,-.42f,0},
    {.0475f,0,0},{0,-.44f,0},{0,-.42f,0}};
  return offsets[i];
}

HumanoidPose MocapRig::retarget(const QByteArray& cameraJson,bool mirror) {
  HumanoidPose result;
  const QJsonObject root=QJsonDocument::fromJson(cameraJson).object();
  const QJsonArray bodies=root.value("bodies").toArray();
  auto missed=[this]{if(++missed_>30) {hasOrigin_=false;hasFloor_=false;hasPrevious_=false;}};
  if(bodies.isEmpty()||!root.value("tracked").toBool()) {missed();return result;}
  const QJsonObject body=bodies.first().toObject();
  const QJsonObject joints=body.value("joints").toObject();
  result.time=root.value("t").toDouble();result.frame=root.value("frame").toInt();
  const QString device=root.value("device").toString();
  const int trackingId=body.value("id").toInt(1);
  if(device!=previousDevice_||trackingId!=previousTrackingId_||mirror!=previousMirror_) {
    hasOrigin_=false;hasFloor_=false;hasPrevious_=false;hasHeadRoll_=false;
    lengths_.fill(0);lengthSamples_.fill(0);
    previousDevice_=device;previousTrackingId_=trackingId;previousMirror_=mirror;
  }
  std::array<QVector3D,19> p{};std::array<bool,19> v{};std::array<float,19> c{};
  auto read=[&](int i,const char* key){p[i]=point(joints,key,v[i],c[i]);};
  read(4,"head");read(5,"shoulderL");read(9,"shoulderR");
  read(7,"elbowL");read(11,"elbowR");read(8,"wristL");read(12,"wristR");
  read(13,"hipL");read(16,"hipR");read(14,"kneeL");read(17,"kneeR");
  read(15,"ankleL");read(18,"ankleR");
  bool earLeftValid=false,earRightValid=false;float earLeftConfidence=0,earRightConfidence=0;
  const QVector3D earLeft=point(joints,"earL",earLeftValid,earLeftConfidence);
  const QVector3D earRight=point(joints,"earR",earRightValid,earRightConfidence);
  if(earLeftValid&&earRightValid&&earLeftConfidence>.2f&&earRightConfidence>.2f&&
     (earLeft-earRight).length()>.08f&&(earLeft-earRight).length()<.28f) {
    const QVector3D skull=midpoint(earLeft,earRight);
    if(!v[4]||(p[4]-skull).length()<.2f) {
      p[4]=v[4]?skull*.7f+p[4]*.3f:skull;
      c[4]=v[4]?std::min(c[4],std::min(earLeftConfidence,earRightConfidence)):
          std::min(earLeftConfidence,earRightConfidence)*.6f;
      v[4]=true;
    }
  }
  const QJsonObject face=root.value("face").toObject();
  const QJsonArray facePoints=face.value("landmarks").toArray();
  bool faceRollObserved=false;
  if(face.value("tracked").toBool()&&face.value("confidence").toDouble()>.6&&
     facePoints.size()==478) {
    const QJsonArray eyeLeft=facePoints[33].toArray(),eyeRight=facePoints[263].toArray();
    if(eyeLeft.size()==3&&eyeRight.size()==3) {
      const float dx=float(eyeRight[0].toDouble()-eyeLeft[0].toDouble());
      const float dy=float(eyeRight[1].toDouble()-eyeLeft[1].toDouble());
      if(std::abs(dx)>.01f) {
        const float measured=std::clamp(float(std::atan2(dy*9.f,dx*16.f)*57.2957795f),-45.f,45.f);
        headRollDegrees_=hasHeadRoll_?headRollDegrees_*.8f+measured*.2f:measured;
        hasHeadRoll_=true;
        faceRollObserved=true;
      }
    }
  }
  if(!faceRollObserved&&earLeftValid&&earRightValid&&
     earLeftConfidence>.2f&&earRightConfidence>.2f&&
     std::abs(earRight.x()-earLeft.x())>.08f) {
    const float measured=std::clamp(float(std::atan2(earRight.y()-earLeft.y(),
        earRight.x()-earLeft.x())*57.2957795f),-45.f,45.f);
    headRollDegrees_=hasHeadRoll_?headRollDegrees_*.85f+measured*.15f:measured;
    hasHeadRoll_=true;
  }
  // The RGB-D tracker accepts three of the four torso landmarks. Reconstruct
  // the fourth from the opposite shoulder/hip separation for the same frame.
  if(!v[5]&&v[9]&&v[13]&&v[16]) {p[5]=p[9]+p[13]-p[16];v[5]=true;c[5]=std::min({c[9],c[13],c[16]})*.35f;}
  if(!v[9]&&v[5]&&v[13]&&v[16]) {p[9]=p[5]+p[16]-p[13];v[9]=true;c[9]=std::min({c[5],c[13],c[16]})*.35f;}
  if(!v[13]&&v[16]&&v[5]&&v[9]) {p[13]=p[16]+p[5]-p[9];v[13]=true;c[13]=std::min({c[16],c[5],c[9]})*.35f;}
  if(!v[16]&&v[13]&&v[5]&&v[9]) {p[16]=p[13]+p[9]-p[5];v[16]=true;c[16]=std::min({c[13],c[5],c[9]})*.35f;}
  if(!(v[5]&&v[9]&&v[13]&&v[16])) {missed();return result;}
  missed_=0;
  p[0]=midpoint(p[13],p[16]);p[3]=midpoint(p[5],p[9]);
  p[1]=p[0]*.6f+p[3]*.4f;p[2]=p[0]*.3f+p[3]*.7f;
  v[0]=v[1]=v[2]=v[3]=true;
  c[0]=c[1]=c[2]=c[3]=std::min({c[5],c[9],c[13],c[16]});
  p[6]=p[5];p[10]=p[9];v[6]=v[5];v[10]=v[9];c[6]=c[5];c[10]=c[9];
  // Keep the hand bone at the wrist. Hand landmarks orient that bone but must
  // not move its joint origin to a fingertip.
  const int segments[][2]={{0,1},{1,2},{2,3},{3,4},{5,7},{7,8},
      {9,11},{11,12},{13,14},{14,15},{16,17},{17,18}};
  for(const auto& segment:segments) {
    const int child=segment[1],parent=segment[0];
    if(!v[parent]||!v[child]||c[parent]<.3f||c[child]<.3f) continue;
    const float measured=(p[child]-p[parent]).length();
    const float nominal=restOffset(child).length();
    if(measured<std::max(.02f,nominal*.45f)||measured>nominal*2.f) continue;
    const int n=lengthSamples_[child];
    lengths_[child]=n==0?measured:lengths_[child]+(measured-lengths_[child])*
        (n<60?1.f/float(n+1):.02f);
    lengthSamples_[child]=n+1;
  }
  auto adaptedOffset=[this](int i) {
    const QVector3D base=restOffset(i);
    return lengthSamples_[i]>0&&base.lengthSquared()>1e-8f?
        base.normalized()*lengths_[i]:base;
  };
  if(!hasOrigin_) {origin_=p[0];hasOrigin_=true;}
  for(int i=0;i<19;++i) {
    const int source=mirror?mirrors[i]:i;
    HumanoidBone& bone=result.bones[i];
    if(v[source]) {
      bone.position=p[source]-QVector3D(origin_.x(),0,origin_.z());
      if(mirror) bone.position.setX(-bone.position.x());
      bone.confidence=c[source];
    } else {
      const int par=parents[i];
      bone.position=hasPrevious_?previousPosition_[i]+QVector3D(0,floor_,0):
          par<0?QVector3D{}:result.bones[par].position+adaptedOffset(i);
    }
    bone.localPosition=parents[i]>=0?adaptedOffset(i):bone.position;
  }
  float floor=std::numeric_limits<float>::infinity();
  for(int i:{15,18}) if(v[i]) floor=std::min(floor,result.bones[i].position.y());
  if(!std::isfinite(floor)) floor=hasFloor_?floor_:result.bones[0].position.y()-.93f;
  if(!hasFloor_) {floor_=floor;hasFloor_=true;}
  else floor_+=std::clamp(floor-floor_,-.04f,.04f)*.2f;
  for(auto& bone:result.bones) bone.position.setY(bone.position.y()-floor_);
  result.bones[0].localPosition=result.bones[0].position;
  // Orient each segment from the parent frame. The bend plane supplies twist
  // for upper limbs; a thumb landmark supplies it for forearms and hands.
  static const int directionChild[]{1,2,3,4,-1,7,7,8,-1,11,11,12,-1,
                                    14,15,-1,17,18,-1};
  auto extra=[&](const char* left,const char* right) {
    bool valid=false;float confidence=0;
    QVector3D value=point(joints,mirror?right:left,valid,confidence);
    if(valid) {
      value-=QVector3D(origin_.x(),0,origin_.z());
      if(mirror) value.setX(-value.x());
      value.setY(value.y()-floor_);
    }
    return std::pair<QVector3D,bool>{value,valid&&confidence>.2f};
  };
  const auto leftThumb=extra("handL_4","handR_4");
  const auto rightThumb=extra("handR_4","handL_4");
  const auto leftTip=extra("handL_8","handR_8");
  const auto rightTip=extra("handR_8","handL_8");
  for(int i=0;i<19;++i) {
    const int child=directionChild[i];
    const int par=parents[i];
    const QQuaternion parentRotation=par>=0?result.bones[par].rotation:QQuaternion{};
    const bool observed=child>=0&&result.bones[i].confidence>0&&
        result.bones[child].confidence>0;
    QQuaternion global=par>=0?parentRotation*previousLocal_[i]:QQuaternion{};
    if(observed) {
      const QVector3D rest=adaptedOffset(child);
      const QVector3D primary=result.bones[child].position-result.bones[i].position;
      global=parentSwing(parentRotation,rest,primary);
      QVector3D secondary,restSide;
      float weight=0;
      if(i==6||i==10||i==13||i==16) {
        const int next=(i==6?8:i==10?12:i==13?15:18);
        if(result.bones[next].confidence>0) {
          secondary=result.bones[next].position-result.bones[child].position;
          restSide=(i==13||i==16)?QVector3D(0,0,-1):QVector3D(0,0,1);
          const QVector3D perpendicular=secondary-QVector3D::dotProduct(
              secondary,primary.normalized())*primary.normalized();
          weight=smoothstep(.08f,.23f,perpendicular.length());
        }
      } else if((i==7&&leftThumb.second)||(i==11&&rightThumb.second)) {
        secondary=(i==7?leftThumb.first:rightThumb.first)-result.bones[child].position;
        restSide={0,0,1};weight=smoothstep(.01f,.03f,secondary.length());
      }
      if(weight>0) {
        const QQuaternion full=frameRotation(rest,restSide,primary,secondary);
        global=QQuaternion::slerp(global,full,weight).normalized();
      }
    }
    if(i<=3&&v[5]&&v[9]&&v[13]&&v[16]) {
      const QVector3D side=i<=1?
          result.bones[16].position-result.bones[13].position:
          result.bones[9].position-result.bones[5].position;
      const QVector3D up=result.bones[3].position-result.bones[0].position;
      if(side.lengthSquared()>1e-6f&&up.lengthSquared()>1e-6f)
        global=torsoFrame(up,side);
    }
    if(i==4&&hasHeadRoll_&&result.bones[i].confidence>0)
      global=(parentRotation*QQuaternion::fromAxisAndAngle(0,0,1,
          mirror?-headRollDegrees_:headRollDegrees_)).normalized();
    if((i==8||i==12)&&result.bones[i].confidence>0) {
      const auto& tip=i==8?leftTip:rightTip;
      const QVector3D primary=tip.first-result.bones[i].position;
      if(tip.second&&primary.lengthSquared()>1e-6f) {
        global=parentSwing(parentRotation,i==8?QVector3D(-1,0,0):QVector3D(1,0,0),primary);
        const auto& thumb=i==8?leftThumb:rightThumb;
        if(thumb.second) {
          const QQuaternion full=frameRotation(i==8?QVector3D(-1,0,0):QVector3D(1,0,0),
              {0,0,1},primary,thumb.first-result.bones[i].position);
          global=QQuaternion::slerp(global,full,.8f).normalized();
        }
      }
    }
    QQuaternion local=(par<0?global:parentRotation.conjugated()*global).normalized();
    if(hasPrevious_&&result.time>previousTime_&&result.time-previousTime_<.25&&
        result.bones[i].confidence>0) {
      const float dt=float(result.time-previousTime_);
      const float dot=std::clamp(std::abs(QQuaternion::dotProduct(previousLocal_[i],local)),0.f,1.f);
      const float angularSpeed=2.f*std::acos(dot)/dt;
      float alpha=1.f-std::exp(-dt*(6.f+std::min(angularSpeed,6.f)*.8f));
      if(result.bones[i].confidence<.2f)alpha*=.5f;
      local=QQuaternion::slerp(previousLocal_[i],local,alpha).normalized();
    } else if(hasPrevious_&&result.bones[i].confidence==0) local=previousLocal_[i];
    previousLocal_[i]=local;
    result.bones[i].localRotation=local;
    result.bones[i].rotation=par<0?local:(parentRotation*local).normalized();
  }
  for(int i=0;i<19;++i)previousPosition_[i]=result.bones[i].position;
  previousTime_=result.time;hasPrevious_=true;
  result.tracked=true;return result;
}

QJsonObject MocapRig::unityJson(const QByteArray& cameraJson,const HumanoidPose& pose,bool mirrored) {
  const QJsonObject source=QJsonDocument::fromJson(cameraJson).object();
  QJsonObject joints;
  for(int i=0;i<HumanoidPose::count;++i) {
    const HumanoidBone& b=pose.bones[i];
    joints[QLatin1String(names[i])]=QJsonObject{{"p",vec(b.position)},
        {"q",quat(b.localRotation)},{"g",quat(b.rotation)},{"c",b.confidence}};
  }
  QJsonArray bodies;
  if(pose.tracked) bodies.append(QJsonObject{{"id",1},{"tracked",true},
      {"confidence",source.value("bodies").toArray().first().toObject().value("confidence")},
      {"joints",joints}});
  QJsonObject output{{"v",1},{"device",source.value("device")},
      {"t",pose.tracked?pose.time:source.value("t").toDouble()},
      {"frame",pose.tracked?pose.frame:source.value("frame").toInt()},
      {"space","unity"},
      {"mirror",mirrored},{"bodies",bodies}};
  if(source.contains("face"))output["face"]=source.value("face");
  if(source.contains("quality"))output["quality"]=source.value("quality");
  return output;
}
