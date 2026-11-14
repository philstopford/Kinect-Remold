#include "MocapNetwork.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr const char* arkitNames[]{
    "eyeBlinkLeft","eyeLookDownLeft","eyeLookInLeft","eyeLookOutLeft","eyeLookUpLeft",
    "eyeSquintLeft","eyeWideLeft","eyeBlinkRight","eyeLookDownRight","eyeLookInRight",
    "eyeLookOutRight","eyeLookUpRight","eyeSquintRight","eyeWideRight","jawForward",
    "jawLeft","jawRight","jawOpen","mouthClose","mouthFunnel","mouthPucker",
    "mouthRight","mouthLeft","mouthSmileLeft","mouthSmileRight","mouthFrownRight",
    "mouthFrownLeft","mouthDimpleLeft","mouthDimpleRight","mouthStretchLeft",
    "mouthStretchRight","mouthRollLower","mouthRollUpper","mouthShrugLower",
    "mouthShrugUpper","mouthPressLeft","mouthPressRight","mouthLowerDownLeft",
    "mouthLowerDownRight","mouthUpperUpLeft","mouthUpperUpRight","browDownLeft",
    "browDownRight","browInnerUp","browOuterUpLeft","browOuterUpRight",
    "cheekPuff","cheekSquintLeft","cheekSquintRight","noseSneerLeft",
    "noseSneerRight","tongueOut"};
void int32(QByteArray& out,quint32 value) {
  out.append(char(value>>24));out.append(char(value>>16));
  out.append(char(value>>8));out.append(char(value));
}
void float32(QByteArray& out,float value) {
  quint32 bits=0;std::memcpy(&bits,&value,4);int32(out,bits);
}
void oscString(QByteArray& out,const QByteArray& value) {
  out.append(value);out.append(char(0));
  while(out.size()%4) out.append(char(0));
}
QByteArray message(const QByteArray& address,const QByteArray& tags,
                   const QByteArray& arguments) {
  QByteArray result;oscString(result,address);oscString(result,","+tags);
  result.append(arguments);return result;
}
void bundleMessage(QByteArray& bundle,const QByteArray& data) {
  int32(bundle,quint32(data.size()));bundle.append(data);
}
QByteArray facePacket(const QJsonObject& root,const QJsonObject& face) {
  const QJsonArray landmarks=face.value("landmarks").toArray();
  if(landmarks.size()!=478)return {};
  QByteArray packed;packed.reserve(478*6);
  auto append16=[&](double value) {
    const quint16 u=quint16(std::lround(std::clamp(value,0.,1.)*65535.));
    packed.append(char(u&255));packed.append(char(u>>8));
  };
  for(const QJsonValue& value:landmarks) {
    const QJsonArray p=value.toArray();if(p.size()!=3)return {};
    for(const QJsonValue& coordinate:p)
      if(!coordinate.isDouble()||!std::isfinite(coordinate.toDouble()))return {};
    append16(p[0].toDouble());append16(p[1].toDouble());
    append16((p[2].toDouble()+1.)*.5);
  }
  return QJsonDocument(QJsonObject{{"v",1},{"kind","face-landmarks"},
      {"person",root.value("person")},
      {"sourceSensor",face.value("sourceSensor")},
      {"device",root.value("device")},{"t",root.value("t")},
      {"frame",root.value("frame")},{"confidence",face.value("confidence")},
      {"format","u16le-uvz"},{"count",478},
      {"data",QString::fromLatin1(packed.toBase64())},
      {"blendshapes",face.value("blendshapes")},
      {"head",face.value("head")},{"eyeGaze",face.value("eyeGaze")}})
      .toJson(QJsonDocument::Compact);
}
}

QByteArray MocapNetwork::encodeOsc(const QByteArray& poseJson,const HumanoidPose* rig) {
  const QJsonDocument document=QJsonDocument::fromJson(poseJson);
  if(!document.isObject()) return {};
  const QJsonObject root=document.object();
  if(root.value("v").toInt()!=1||root.value("space").toString()!="camera") return {};
  QByteArray bundle;oscString(bundle,"#bundle");
  int32(bundle,0);int32(bundle,1); // immediate time tag
  const QJsonArray bodies=root.value("bodies").toArray();
  const QJsonObject body=bodies.isEmpty()?QJsonObject{}:bodies.first().toObject();
  QByteArray args;
  int32(args,quint32(root.value("frame").toDouble()));
  float32(args,float(root.value("t").toDouble()));
  int32(args,quint32(body.value("id").toInt()));
  int32(args,body.value("tracked").toBool()?1:0);
  float32(args,0);float32(args,-1);float32(args,0);
  bundleMessage(bundle,message("/synkinect/frame","ifiifff",args));
  const QJsonObject joints=body.value("joints").toObject();
  for(auto it=joints.begin();it!=joints.end();++it) {
    const QJsonObject joint=it.value().toObject();
    const QJsonArray position=joint.value("p").toArray();
    if(position.size()!=3) continue;
    QByteArray jointArgs;oscString(jointArgs,it.key().toUtf8());
    for(int i=0;i<3;++i) float32(jointArgs,float(position[i].toDouble()));
    float32(jointArgs,float(joint.value("c").toDouble()));
    bundleMessage(bundle,message("/synkinect/joint","sffff",jointArgs));
  }
  if(rig&&rig->tracked) for(int i=0;i<HumanoidPose::count;++i) {
    const HumanoidBone& bone=rig->bones[i];QByteArray args;
    oscString(args,MocapRig::name(i));
    float32(args,bone.position.x());float32(args,bone.position.y());
    float32(args,bone.position.z());
    float32(args,bone.rotation.x());float32(args,bone.rotation.y());
    float32(args,bone.rotation.z());float32(args,bone.rotation.scalar());
    bundleMessage(bundle,message("/synkinect/bone","sfffffff",args));
  }
  return bundle;
}

QByteArray MocapNetwork::encodeVmc(const HumanoidPose& rig) {
  if(!rig.tracked) return {};
  QByteArray bundle;oscString(bundle,"#bundle");int32(bundle,0);int32(bundle,1);
  QByteArray ok;int32(ok,1);bundleMessage(bundle,message("/VMC/Ext/OK","i",ok));
  QByteArray tick;float32(tick,float(rig.time));
  bundleMessage(bundle,message("/VMC/Ext/T","f",tick));
  QByteArray root;oscString(root,"root");
  for(int i=0;i<3;++i)float32(root,0);
  for(int i=0;i<3;++i)float32(root,0);float32(root,1);
  bundleMessage(bundle,message("/VMC/Ext/Root/Pos","sfffffff",root));
  for(int i=0;i<HumanoidPose::count;++i) {
    const HumanoidBone& bone=rig.bones[i];QByteArray args;
    oscString(args,MocapRig::name(i));
    float32(args,bone.localPosition.x());float32(args,bone.localPosition.y());
    float32(args,bone.localPosition.z());
    float32(args,bone.localRotation.x());float32(args,bone.localRotation.y());
    float32(args,bone.localRotation.z());float32(args,bone.localRotation.scalar());
    bundleMessage(bundle,message("/VMC/Ext/Bone/Pos","sfffffff",args));
  }
  return bundle;
}

QByteArray MocapNetwork::encodeFaceCap(const QJsonObject& face) {
  if(!face.value("tracked").toBool())return {};
  const QJsonObject coefficients=face.value("blendshapes").toObject();
  if(coefficients.size()<50)return {};
  QByteArray bundle;oscString(bundle,"#bundle");int32(bundle,0);int32(bundle,1);
  for(int i=0;i<52;++i) {
    QByteArray args;int32(args,quint32(i));
    const float weight=i==51?0.f:float(coefficients.value(QLatin1String(arkitNames[i])).toDouble());
    float32(args,std::clamp(weight,0.f,1.f));
    bundleMessage(bundle,message("/W","if",args));
  }
  return bundle;
}

QByteArray MocapNetwork::encodeVmcFace(const QJsonObject& face) {
  if(!face.value("tracked").toBool())return {};
  const QJsonObject coefficients=face.value("blendshapes").toObject();
  if(coefficients.size()<50)return {};
  QByteArray bundle;oscString(bundle,"#bundle");int32(bundle,0);int32(bundle,1);
  for(int i=0;i<51;++i) {
    QByteArray args;oscString(args,arkitNames[i]);
    float32(args,float(std::clamp(coefficients.value(QLatin1String(arkitNames[i])).toDouble(),0.,1.)));
    bundleMessage(bundle,message("/VMC/Ext/Blend/Val","sf",args));
  }
  bundleMessage(bundle,message("/VMC/Ext/Blend/Apply","",{}));
  return bundle;
}

bool MocapNetwork::send(const QByteArray& poseJson,const HumanoidPose* rig,
                        const QByteArray& jsonOutput) {
  error_.clear();
  if(jsonEnabled_) {
    const QByteArray& payload=jsonOutput.isEmpty()?poseJson:jsonOutput;
    QJsonObject root=QJsonDocument::fromJson(payload).object();
    const QJsonArray tracks=root.value("tracks").toArray();
    auto sendPerson=[&](QJsonObject frame) {
      const QJsonObject face=frame.value("face").toObject();
      frame.remove("face");frame.remove("tracks");
      const QByteArray bodyPayload=QJsonDocument(frame).toJson(QJsonDocument::Compact);
      if(socket_.writeDatagram(bodyPayload,jsonHost_,jsonPort_)!=bodyPayload.size())
        error_=socket_.errorString();
      else ++jsonPackets_;
      if(face.value("tracked").toBool()) {
        const QByteArray compact=facePacket(frame,face);
        if(compact.isEmpty())error_="Invalid facial landmark packet";
        else if(socket_.writeDatagram(compact,jsonHost_,jsonPort_)!=compact.size())
          error_=socket_.errorString();
        else ++jsonPackets_;
      }
    };
    if(tracks.isEmpty())sendPerson(root);
    else for(const QJsonValue& value:tracks) {
      const QJsonObject track=value.toObject();
      QJsonObject frame{{"v",1},{"person",track.value("id")},
          {"device",track.value("sensor")},{"t",root.value("t")},
          {"frame",root.value("frame")},{"space",track.value("space")},
          {"tracked",track.value("tracked")},{"observed",track.value("observed")},
          {"bodies",track.value("bodies")},{"face",track.value("face")}};
      sendPerson(frame);
    }
  }
  if(oscEnabled_) {
    const QByteArray packet=encodeOsc(poseJson,rig);
    if(packet.isEmpty()) error_="Invalid pose for OSC";
    else if(socket_.writeDatagram(packet,oscHost_,oscPort_)!=packet.size())
      error_=socket_.errorString();
    else ++oscPackets_;
  }
  if(vmcEnabled_&&rig&&rig->tracked) {
    const QByteArray packet=encodeVmc(*rig);
    if(socket_.writeDatagram(packet,vmcHost_,vmcPort_)!=packet.size())
      error_=socket_.errorString();
    else ++vmcPackets_;
  }
  const QJsonObject face=QJsonDocument::fromJson(poseJson).object().value("face").toObject();
  if(vmcEnabled_&&!face.isEmpty()) {
    const QByteArray packet=encodeVmcFace(face);
    if(packet.isEmpty())error_="Invalid VMC facial coefficients";
    else if(socket_.writeDatagram(packet,vmcHost_,vmcPort_)!=packet.size())
      error_=socket_.errorString();
    else ++facePackets_;
  }
  if(faceCapEnabled_&&!face.isEmpty()) {
    const QByteArray packet=encodeFaceCap(face);
    if(packet.isEmpty())error_="Invalid Face Cap coefficients";
    else if(socket_.writeDatagram(packet,faceHost_,facePort_)!=packet.size())
      error_=socket_.errorString();
    else ++facePackets_;
  }
  return error_.isEmpty();
}
