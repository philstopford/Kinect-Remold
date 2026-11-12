#include "MocapClipProcessor.hpp"
#include "MocapBvh.hpp"
#include "MocapRig.hpp"
#include "ArticulatedPoseFit.hpp"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QQuaternion>
#include <QPointF>
#include <QSaveFile>
#include <QTemporaryFile>
#include <QVector3D>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace {
QJsonObject readFrame(QFile& file) {
  if(file.property("compact").isNull()) {
    const bool compact=file.peek(4)=="RMC1";
    file.setProperty("compact",compact);
    if(compact)file.read(4);
  }
  if(file.property("compact").toBool()) {
    const QByteArray header=file.read(4);
    if(header.isEmpty()&&file.atEnd())return {};
    if(header.size()!=4){file.setProperty("decodeError","Truncated compact frame header");return {};}
    const auto* p=reinterpret_cast<const uchar*>(header.constData());
    const quint32 size=quint32(p[0])|(quint32(p[1])<<8)|(quint32(p[2])<<16)|(quint32(p[3])<<24);
    if(size<5||size>8u*1024u*1024u){file.setProperty("decodeError","Invalid compact frame size");return {};}
    const QByteArray packed=file.read(size);
    if(packed.size()!=size){file.setProperty("decodeError","Truncated compact frame");return {};}
    const QByteArray line=qUncompress(packed);
    if(line.isEmpty()||line.size()>8*1024*1024){file.setProperty("decodeError","Invalid compressed frame");return {};}
    const QJsonObject frame=QJsonDocument::fromJson(line).object();
    if(frame.isEmpty())file.setProperty("decodeError","Invalid compact JSON frame");
    return frame;
  }
  while(!file.atEnd()) {
    const QByteArray line=file.readLine();
    if(line.trimmed().isEmpty())continue;
    if(line.size()>8*1024*1024){file.setProperty("decodeError","JSONL frame exceeds 8 MiB");return {};}
    const auto doc=QJsonDocument::fromJson(line);
    if(doc.isObject())return doc.object();
    file.setProperty("decodeError","Invalid JSONL frame");return {};
  }
  return {};
}
QJsonValue interpolate(const QJsonValue& a,const QJsonValue& b,double w) {
  if(a.isDouble()&&b.isDouble())return a.toDouble()*(1-w)+b.toDouble()*w;
  if(a.isArray()&&b.isArray()) {
    const auto x=a.toArray(),y=b.toArray();if(x.size()!=y.size())return a;
    QJsonArray out;for(int i=0;i<x.size();++i)out.append(interpolate(x[i],y[i],w));return out;
  }
  if(a.isObject()&&b.isObject()) {
    QJsonObject out=a.toObject(),other=b.toObject();
    for(auto it=out.begin();it!=out.end();++it)if(other.contains(it.key())) {
      if(it.key()=="q"&&it.value().toArray().size()==4&&other.value(it.key()).toArray().size()==4) {
        const auto x=it.value().toArray(),y=other.value(it.key()).toArray();
        const QQuaternion qa(x[3].toDouble(),x[0].toDouble(),x[1].toDouble(),x[2].toDouble());
        const QQuaternion qb(y[3].toDouble(),y[0].toDouble(),y[1].toDouble(),y[2].toDouble());
        const auto q=QQuaternion::slerp(qa.normalized(),qb.normalized(),float(w));
        it.value()=QJsonArray{q.x(),q.y(),q.z(),q.scalar()};
      } else it.value()=interpolate(it.value(),other.value(it.key()),w);
    }
    return out;
  }
  return a;
}
QVector3D rootPosition(const QJsonObject& frame) {
  const auto bodies=frame.value("bodies").toArray();if(bodies.isEmpty())return {};
  const auto joints=bodies[0].toObject().value("joints").toObject();
  const auto l=joints.value("hipL").toObject().value("p").toArray();
  const auto r=joints.value("hipR").toObject().value("p").toArray();
  if(l.size()!=3||r.size()!=3)return {};
  return {float((l[0].toDouble()+r[0].toDouble())*.5),
      float((l[1].toDouble()+r[1].toDouble())*.5),
      float((l[2].toDouble()+r[2].toDouble())*.5)};
}
struct FaceRegistration {
  double scale=1,cosine=1,sine=0,lastAnchorTime=0,lastOutputTime=0;
  QPointF translation;
  bool valid=false;
  QPointF map(const QPointF& point) const {
    return {translation.x()+scale*(cosine*point.x()-sine*point.y()),
            translation.y()+scale*(sine*point.x()+cosine*point.y())};
  }
  void anchor(const QPointF& source,const QPointF& target) {
    translation=target-QPointF(scale*(cosine*source.x()-sine*source.y()),
                                scale*(sine*source.x()+cosine*source.y()));
  }
};
QJsonObject attachFace(const QJsonObject& bodyFrame,const QJsonObject& faceFrame,
    FaceRegistration& previous,double time) {
  if(previous.valid&&time-previous.lastOutputTime>.5)previous.valid=false;
  const QJsonObject captured=faceFrame.value("face").toObject();
  QJsonObject attached=bodyFrame.value("face").toObject();
  attached["tracked"]=captured.value("tracked");
  attached["confidence"]=captured.value("confidence");
  attached["blendshapes"]=captured.value("blendshapes");
  attached.remove("landmarks");attached.remove("head");
  attached["registration"]=QJsonObject{{"state","expressions-only"}};
  const QJsonArray bodies=bodyFrame.value("bodies").toArray();
  if(bodies.isEmpty()||!captured.value("tracked").toBool())return attached;
  const QJsonObject joints=bodies[0].toObject().value("joints").toObject();
  const QJsonArray source=captured.value("landmarks").toArray();
  auto uv=[&](const char* name) {
    const QJsonObject j=joints.value(name).toObject();
    return j.value("c").toDouble()>=.25?j.value("uv").toArray():QJsonArray{};
  };
  const QJsonArray nose=uv("nose"),earL=uv("earL"),earR=uv("earR");
  if(source.size()!=478||nose.size()!=2)return attached;
  const QJsonArray sourceNose=source[1].toArray();
  QJsonArray sourceEarA=source[234].toArray(),sourceEarB=source[454].toArray();
  if(sourceNose.size()!=3||sourceEarA.size()!=3||sourceEarB.size()!=3)return attached;
  auto xy=[](const QJsonArray& p){return QPointF(p[0].toDouble(),p[1].toDouble());};
  if(sourceEarA[0].toDouble()>sourceEarB[0].toDouble())std::swap(sourceEarA,sourceEarB);
  const QPointF sourceN=xy(sourceNose),targetN=xy(nose);
  FaceRegistration fit=previous;
  QString mode;
  double rms=0;
  if(earL.size()==2&&earR.size()==2) {
    QPointF targetA=xy(earL),targetB=xy(earR);
    if(targetA.x()>targetB.x())std::swap(targetA,targetB);
    const std::array<QPointF,3> src{sourceN,xy(sourceEarA),xy(sourceEarB)};
    const std::array<QPointF,3> dst{targetN,targetA,targetB};
    QPointF srcMean,dstMean;
    for(int i=0;i<3;++i){srcMean+=src[i];dstMean+=dst[i];}
    srcMean/=3.;dstMean/=3.;
    double dot=0,cross=0,denom=0;
    for(int i=0;i<3;++i) {
      const QPointF a=src[i]-srcMean,b=dst[i]-dstMean;
      dot+=a.x()*b.x()+a.y()*b.y();
      cross+=a.x()*b.y()-a.y()*b.x();
      denom+=a.x()*a.x()+a.y()*a.y();
    }
    const double magnitude=std::hypot(dot,cross);
    if(denom>1e-5&&magnitude>1e-6) {
      fit.scale=magnitude/denom;fit.cosine=dot/magnitude;fit.sine=cross/magnitude;
      fit.anchor(srcMean,dstMean);
      double error=0;
      for(int i=0;i<3;++i) {
        const QPointF d=fit.map(src[i])-dst[i];error+=d.x()*d.x()+d.y()*d.y();
      }
      rms=std::sqrt(error/3.);
      if(fit.scale>=.12&&fit.scale<=4.&&rms<=.055)mode="aligned";
    }
  }
  if(mode.isEmpty()&&((earL.size()==2)!=(earR.size()==2))) {
    const bool left=earL.size()==2;
    const QPointF sourceE=xy(left?sourceEarB:sourceEarA);
    const QPointF targetE=xy(left?earL:earR);
    const QPointF a=sourceE-sourceN,b=targetE-targetN;
    const double al=std::hypot(a.x(),a.y()),bl=std::hypot(b.x(),b.y());
    if(al>.015&&bl>.015) {
      const double scale=bl/al;
      const double dot=a.x()*b.x()+a.y()*b.y();
      const double cross=a.x()*b.y()-a.y()*b.x();
      const double magnitude=al*bl;
      if(scale>=.12&&scale<=4.&&(!previous.valid||
          (scale>=previous.scale*.5&&scale<=previous.scale*2.))) {
        fit.scale=scale;fit.cosine=dot/magnitude;fit.sine=cross/magnitude;
        fit.anchor(sourceN,targetN);mode="aligned-partial";
      }
    }
  }
  if(mode.isEmpty()&&previous.valid&&time-previous.lastAnchorTime<.5) {
    fit=previous;fit.anchor(sourceN,targetN);mode="aligned-temporal";
  }
  if(mode.isEmpty())return attached;
  if(mode!="aligned-temporal"&&previous.valid) {
    const double dt=time-previous.lastOutputTime;
    if(dt>.001&&dt<.5) {
      const double maxLogScale=.04+1.5*dt;
      const double change=std::clamp(std::log(fit.scale/previous.scale),
          -maxLogScale,maxLogScale);
      fit.scale=previous.scale*std::exp(change);
      const double oldAngle=std::atan2(previous.sine,previous.cosine);
      const double newAngle=std::atan2(fit.sine,fit.cosine);
      const double delta=std::atan2(std::sin(newAngle-oldAngle),std::cos(newAngle-oldAngle));
      const double angle=oldAngle+std::clamp(delta,-(.05+2.*dt),.05+2.*dt);
      fit.cosine=std::cos(angle);fit.sine=std::sin(angle);
      fit.anchor(sourceN,targetN);
    }
  }
  fit.valid=true;
  if(mode!="aligned-temporal")fit.lastAnchorTime=time;
  fit.lastOutputTime=time;previous=fit;
  QJsonArray mapped;
  for(const QJsonValue& value:source) {
    const QJsonArray p=value.toArray();
    if(p.size()!=3){mapped.append(p);continue;}
    const QPointF q=fit.map(xy(p));
    mapped.append(QJsonArray{q.x(),q.y(),p[2].toDouble()*fit.scale});
  }
  attached["landmarks"]=mapped;
  QJsonObject head;
  const QJsonArray headP=joints.value("head").toObject().value("p").toArray();
  if(headP.size()==3)head["pCamera"]=headP;
  if(!head.isEmpty())attached["head"]=head;
  QJsonObject registration{{"state",mode},{"scale",fit.scale},
      {"anchors",mode=="aligned"?3:mode=="aligned-partial"?2:1}};
  if(mode=="aligned")registration["anchorRms"]=rms;
  if(mode=="aligned-temporal")registration["holdMs"]=(time-fit.lastAnchorTime)*1000.;
  attached["registration"]=registration;
  return attached;
}
struct PoseSignature {
  int index=0;
  double time=0;
  QVector3D root;
  std::array<QVector3D,8> local{},velocity{};
  std::array<bool,8> valid{};
  std::array<float,2> footY{},footSpeed{};
};
PoseSignature signature(const QJsonObject& frame,int index,const PoseSignature* previous) {
  PoseSignature s;s.index=index;s.time=frame.value("t").toDouble();s.root=rootPosition(frame);
  const QJsonArray bodies=frame.value("bodies").toArray();
  if(bodies.isEmpty()||s.root.isNull())return s;
  const QJsonObject joints=bodies[0].toObject().value("joints").toObject();
  const char* names[]{"shoulderL","shoulderR","wristL","wristR",
      "kneeL","kneeR","ankleL","ankleR"};
  for(int i=0;i<8;++i) {
    const QJsonObject joint=joints.value(names[i]).toObject();
    const QJsonArray p=joint.value("p").toArray();
    if(p.size()!=3||joint.value("c").toDouble()<.2)continue;
    const QVector3D position(float(p[0].toDouble()),float(p[1].toDouble()),float(p[2].toDouble()));
    s.local[i]=position-s.root;s.valid[i]=true;
    if(i>=6)s.footY[i-6]=position.y();
    const double dt=previous?s.time-previous->time:0;
    if(previous&&previous->valid[i]&&dt>.001&&dt<.2) {
      s.velocity[i]=(s.local[i]-previous->local[i])/float(dt);
      if(i>=6) {
        const QVector3D old=previous->local[i]+previous->root;
        s.footSpeed[i-6]=(position-old).length()/float(dt);
      }
    }
  }
  return s;
}
double seamScore(const PoseSignature& a,const PoseSignature& b,float floorY,int total) {
  double score=0;int matches=0;
  for(int i=0;i<8;++i)if(a.valid[i]&&b.valid[i]) {
    const QVector3D delta=a.local[i]-b.local[i];
    const double weight=i>=6?2.:i>=4?1.5:1.;
    score+=weight*delta.lengthSquared()+.012*(a.velocity[i]-b.velocity[i]).lengthSquared();
    ++matches;
  }
  if(matches<4)return 1e9;
  if(std::isfinite(floorY))for(int foot=0;foot<2;++foot)if(a.valid[6+foot]&&b.valid[6+foot]) {
    const bool plantedA=floorY-a.footY[foot]<.07f&&a.footSpeed[foot]<.3f;
    const bool plantedB=floorY-b.footY[foot]<.07f&&b.footSpeed[foot]<.3f;
    if(plantedA!=plantedB)score+=.6;
  }
  score/=matches;
  score+=.01*(a.index+(total-1-b.index))/std::max(1,total);
  return score;
}
QJsonObject stabilizeFoot(QJsonObject frame,const QJsonObject& observed,
    float floorY,int side,QVector3D& anchor,QVector3D& previous,
    double& previousTime,bool& planted,int& heldFrames) {
  const QJsonArray bodies=frame.value("bodies").toArray(),rawBodies=observed.value("bodies").toArray();
  if(bodies.isEmpty()||rawBodies.isEmpty()||!std::isfinite(floorY))return frame;
  QJsonObject body=bodies[0].toObject(),joints=body.value("joints").toObject();
  const QJsonObject raw=rawBodies[0].toObject().value("joints").toObject();
  const char* ankle=side==0?"ankleL":"ankleR";
  const char* knee=side==0?"kneeL":"kneeR";
  const char* hip=side==0?"hipL":"hipR";
  auto point=[&](const QJsonObject& source,const char* name,QVector3D& value) {
    const QJsonArray p=source.value(name).toObject().value("p").toArray();
    if(p.size()!=3)return false;
    value={float(p[0].toDouble()),float(p[1].toDouble()),float(p[2].toDouble())};return true;
  };
  QVector3D observedAnkle;
  if(!point(raw,ankle,observedAnkle)||raw.value(ankle).toObject().value("c").toDouble()<.25) {
    planted=false;return frame;
  }
  const double time=frame.value("t").toDouble(),dt=time-previousTime;
  const float speed=dt>.001&&dt<.2?(observedAnkle-previous).length()/float(dt):0;
  previous=observedAnkle;previousTime=time;
  const bool nearFloor=floorY-observedAnkle.y()<(planted?.09f:.055f);
  const bool still=speed<(planted?.45f:.22f);
  if(!nearFloor||!still){planted=false;return frame;}
  QVector3D hipP,kneeP,ankleP;
  if(!point(joints,hip,hipP)||!point(joints,knee,kneeP)||!point(joints,ankle,ankleP))return frame;
  if(!planted){anchor=ankleP;planted=true;}
  const float upper=(kneeP-hipP).length(),lower=(ankleP-kneeP).length();
  QVector3D axis=anchor-hipP;const float distance=axis.length();
  if(distance<.05f||distance>upper+lower+.025f||distance<std::abs(upper-lower)-.025f) {
    planted=false;return frame;
  }
  axis/=distance;
  const float along=(upper*upper-lower*lower+distance*distance)/(2*distance);
  const float radius=std::sqrt(std::max(0.f,upper*upper-along*along));
  QVector3D bend=kneeP-(hipP+axis*along);bend-=axis*QVector3D::dotProduct(bend,axis);
  if(bend.length()<.001f)bend=QVector3D::crossProduct(axis,{0,0,1});
  if(bend.length()<.001f)bend={1,0,0};
  const QVector3D solvedKnee=hipP+axis*along+bend.normalized()*radius;
  auto set=[&](const char* name,const QVector3D& p) {
    QJsonObject j=joints.value(name).toObject();j["p"]=QJsonArray{p.x(),p.y(),p.z()};joints[name]=j;
  };
  set(knee,solvedKnee);set(ankle,anchor);body["joints"]=joints;
  ++heldFrames;
  QJsonArray changed=bodies;changed[0]=body;frame["bodies"]=changed;
  return frame;
}
QJsonObject shiftBody(QJsonObject frame,const QVector3D& shift) {
  QJsonArray bodies=frame.value("bodies").toArray();
  for(int n=0;n<bodies.size();++n) {
    QJsonObject body=bodies[n].toObject(),joints=body.value("joints").toObject();
    for(auto it=joints.begin();it!=joints.end();++it) {
      QJsonObject joint=it.value().toObject();QJsonArray p=joint.value("p").toArray();
      if(p.size()!=3)continue;
      joint["p"]=QJsonArray{p[0].toDouble()-shift.x(),p[1].toDouble()-shift.y(),p[2].toDouble()-shift.z()};
      it.value()=joint;
    }
    body["joints"]=joints;bodies[n]=body;
  }
  frame["bodies"]=bodies;return frame;
}
QJsonObject syncFaceHead(QJsonObject frame) {
  QJsonObject face=frame.value("face").toObject();
  if(face.isEmpty())return frame;
  const QJsonArray bodies=frame.value("bodies").toArray();
  if(bodies.isEmpty())return frame;
  const QJsonArray headPoint=bodies[0].toObject().value("joints").toObject()
      .value("head").toObject().value("p").toArray();
  if(headPoint.size()!=3)return frame;
  QJsonObject head=face.value("head").toObject();head["pCamera"]=headPoint;
  face["head"]=head;frame["face"]=face;return frame;
}
bool writeFrame(QIODevice& file,const QJsonObject& frame) {
  const QByteArray json=QJsonDocument(frame).toJson(QJsonDocument::Compact);
  if(file.property("compact").toBool()) {
    const QByteArray packed=qCompress(json,3);
    const quint32 n=quint32(packed.size());QByteArray prefix(4,0);
    for(int i=0;i<4;++i)prefix[i]=char((n>>(i*8))&255);
    return file.write(prefix)==4&&file.write(packed)==packed.size();
  }
  const QByteArray line=json+'\n';return file.write(line)==line.size();
}
}

MocapClipResult MocapClipProcessor::process(const MocapClipOptions& opt) {
  MocapClipResult result;
  if(opt.source.isEmpty()||opt.output.isEmpty()) {result.error="Select source and output files";return result;}
  QFile source(opt.source);if(!source.open(QIODevice::ReadOnly)){result.error=source.errorString();return result;}
  QFile face(opt.face);
  if(!opt.face.isEmpty()&&!face.open(QIODevice::ReadOnly)){result.error=face.errorString();return result;}
  if(opt.trimEnd>0&&opt.trimEnd<=opt.trimStart){result.error="Trim end must follow trim start";return result;}
  QTemporaryFile stage(QFileInfo(opt.output).absolutePath()+"/.remold-edit-XXXXXX");
  if(!stage.open()){result.error=stage.errorString();return result;}
  QJsonObject first=readFrame(source);
  if(first.isEmpty()){result.error="No readable pose frames";return result;}
  const double sourceStart=first.value("t").toDouble();
  QJsonObject faceNext=opt.face.isEmpty()?QJsonObject{}:readFrame(face);
  if(!opt.face.isEmpty()&&faceNext.isEmpty()){result.error="Face recording has no readable frames";return result;}
  QJsonObject currentFace;
  FaceRegistration faceRegistration;
  const double faceStart=faceNext.value("t").toDouble();
  QJsonObject previous;
  double previousTime=0,firstTime=0,lastTime=0;
  QVector3D firstRoot,lastRoot;
  std::vector<PoseSignature> frontSignatures;
  std::deque<PoseSignature> tailSignatures;
  PoseSignature previousSignature;bool hasSignature=false;
  std::array<int,600> floorBins{};int floorSamples=0;
  do {
    const double t=first.value("t").toDouble();
    if(opt.trimEnd>0&&t-sourceStart>opt.trimEnd)break;
    if(t-sourceStart>=opt.trimStart&&
       (opt.trimEnd<=0||t-sourceStart<=opt.trimEnd)) {
      if(!opt.face.isEmpty()) {
        const double target=t-sourceStart-opt.faceOffset;
        while(!faceNext.isEmpty()&&faceNext.value("t").toDouble()-faceStart<=target) {
          currentFace=faceNext;faceNext=readFrame(face);
        }
        if(!currentFace.isEmpty()&&currentFace.value("face").isObject()) {
          first["face"]=attachFace(first,currentFace,faceRegistration,t);
        }
      }
      if(opt.smoothing>0&&!previous.isEmpty()) {
        const double dt=std::clamp(t-previousTime,.001,.2);
        const double rate=18.-opt.smoothing*.15;
        const double alpha=1.-std::exp(-dt*rate);
        QJsonArray bodies=first.value("bodies").toArray();
        const QJsonArray oldBodies=previous.value("bodies").toArray();
        if(!bodies.isEmpty()&&!oldBodies.isEmpty()) {
          QJsonObject body=bodies[0].toObject(),joints=body.value("joints").toObject();
          const QJsonObject old=oldBodies[0].toObject().value("joints").toObject();
          for(auto it=joints.begin();it!=joints.end();++it) {
            QJsonObject j=it.value().toObject(),prior=old.value(it.key()).toObject();
            if(j.value("p").toArray().size()==3&&prior.value("p").toArray().size()==3) {
              j["p"]=interpolate(prior.value("p"),j.value("p"),alpha);it.value()=j;
            }
          }
          body["joints"]=joints;bodies[0]=body;first["bodies"]=bodies;
        }
        if(first.value("face").isObject()&&previous.value("face").isObject())
          first["face"]=interpolate(previous.value("face"),first.value("face"),alpha);
      }
      if(result.frames==0){firstTime=t;firstRoot=rootPosition(first);}
      lastTime=t;lastRoot=rootPosition(first);
      const PoseSignature sig=signature(first,result.frames,
          hasSignature?&previousSignature:nullptr);
      previousSignature=sig;hasSignature=true;
      if(frontSignatures.size()<400)frontSignatures.push_back(sig);
      tailSignatures.push_back(sig);if(tailSignatures.size()>400)tailSignatures.pop_front();
      for(int foot=0;foot<2;++foot)if(sig.valid[6+foot]) {
        const int bin=std::clamp(int(std::floor((sig.footY[foot]+2.f)*100.f)),0,599);
        ++floorBins[size_t(bin)];++floorSamples;
      }
      if(!writeFrame(stage,first)){result.error=stage.errorString();return result;}
      previous=first;previousTime=t;++result.frames;
    }
    first=readFrame(source);
  } while(!first.isEmpty());
  if(source.property("decodeError").isValid()) {result.error=source.property("decodeError").toString();return result;}
  if(face.property("decodeError").isValid()) {result.error=face.property("decodeError").toString();return result;}
  if(result.frames==0){result.error="Trim contains no frames";return result;}
  QSaveFile output(opt.output);
  if(!output.open(QIODevice::WriteOnly)){result.error=output.errorString();return result;}
  const bool compactOutput=opt.output.endsWith(".rmclip",Qt::CaseInsensitive);
  output.setProperty("compact",compactOutput);
  if(compactOutput&&output.write("RMC1",4)!=4){result.error=output.errorString();output.cancelWriting();return result;}
  stage.flush();stage.seek(0);
  const double duration=std::max(.001,lastTime-firstTime);
  const int overlap=opt.loopBlend>0?std::clamp(int(std::lround(
      opt.loopBlend*std::max(1.,double(result.frames-1)/duration))),2,300):0;
  const bool loop=overlap>0&&result.frames>overlap*2+2;
  if(overlap>0&&!loop){result.error="Loop blend is too long for the trimmed clip";output.cancelWriting();return result;}
  float floorY=std::numeric_limits<float>::quiet_NaN();
  if(floorSamples>=20) {
    const int target=int(std::ceil(floorSamples*.95));int seen=0;
    for(int bin=0;bin<600;++bin) {
      seen+=floorBins[size_t(bin)];
      if(seen>=target){floorY=float(bin)/100.f-2.f;break;}
    }
  }
  int startIndex=0,endIndex=result.frames-1;
  if(loop) {
    double best=1e9;
    const int candidateWindow=std::min(120,std::max(30,result.frames/4));
    const int startCandidates=std::min({candidateWindow,
        int(frontSignatures.size())-overlap});
    for(int s=0;s<std::max(0,startCandidates);++s) {
      const PoseSignature& opening=frontSignatures[size_t(s+overlap-1)];
      for(const PoseSignature& ending:tailSignatures) {
        if(ending.index<result.frames-candidateWindow||ending.index-s<overlap*2+2)continue;
        const double score=seamScore(opening,ending,floorY,result.frames);
        if(score<best){best=score;startIndex=s;endIndex=ending.index;}
      }
    }
    result.seamScore=best;
    if(best>=1e8)result.warning="No reliable body seam was found; clip edges were blended.";
    result.seamStartFrame=startIndex;result.seamEndFrame=endIndex;
    if(startIndex<int(frontSignatures.size())) {
      firstTime=frontSignatures[size_t(startIndex)].time;
      firstRoot=frontSignatures[size_t(startIndex)].root;
    }
    for(const PoseSignature& ending:tailSignatures)if(ending.index==endIndex) {
      lastTime=ending.time;lastRoot=ending.root;break;
    }
    result.frames=endIndex-startIndex+1;
  }
  const bool applyInPlace=loop&&opt.inPlaceLoop&&!firstRoot.isNull()&&!lastRoot.isNull();
  if(loop&&opt.inPlaceLoop&&!applyInPlace) {
    if(!result.warning.isEmpty())result.warning+=' ';
    result.warning+="Root travel could not be removed without observed hips.";
  }
  std::vector<QJsonObject> head;
  auto seekStart=[&] {
    stage.seek(0);
    for(int i=0;i<startIndex;++i)if(readFrame(stage).isEmpty())return false;
    return true;
  };
  if(!seekStart()){result.error="Cannot seek to loop start";output.cancelWriting();return result;}
  if(loop) {
    head.reserve(size_t(overlap));
    size_t headBytes=0;
    for(int i=0;i<overlap;++i) {
      head.push_back(readFrame(stage));
      if(head.back().isEmpty()){result.error="Staged loop head is incomplete";output.cancelWriting();return result;}
      headBytes+=size_t(QJsonDocument(head.back()).toJson(QJsonDocument::Compact).size());
      if(headBytes>32u*1024u*1024u) {
        result.error="Loop blend exceeds the 32 MiB working-memory limit";
        output.cancelWriting();return result;
      }
    }
    if(!seekStart()){result.error="Cannot rewind loop start";output.cancelWriting();return result;}
  }
  const QVector3D drift=lastRoot-firstRoot;
  const double selectedDuration=std::max(.001,lastTime-firstTime);
  ArticulatedPoseFit finalFit;
  std::array<QVector3D,2> plantAnchor{},previousFoot{};
  std::array<double,2> previousFootTime{};
  std::array<bool,2> planted{};
  for(int i=0;i<result.frames;++i) {
    QJsonObject frame=readFrame(stage);if(frame.isEmpty()){result.error="Staged clip is incomplete";output.cancelWriting();return result;}
    const QJsonObject observed=frame;
    if(loop&&i<overlap)continue;
    if(applyInPlace)frame=shiftBody(frame,drift*float(
        (frame.value("t").toDouble()-firstTime)/selectedDuration));
    if(loop&&i>=result.frames-overlap) {
      const int j=i-(result.frames-overlap);
      QJsonObject target=head[size_t(j)];
      if(applyInPlace)target=shiftBody(target,drift*float(
          (target.value("t").toDouble()-firstTime)/selectedDuration));
      const double u=double(j+1)/overlap;
      const double w=u*u*(3.-2.*u);
      // Preserve source frame time and metadata while blending pose and face.
      frame["bodies"]=interpolate(frame.value("bodies"),target.value("bodies"),w);
      if(frame.value("face").isObject()&&target.value("face").isObject())
        frame["face"]=interpolate(frame.value("face"),target.value("face"),w);
    }
    if(opt.smoothing>0||loop)frame=finalFit.fit(frame);
    if(applyInPlace)for(int foot=0;foot<2;++foot)
      frame=stabilizeFoot(frame,observed,floorY,foot,plantAnchor[size_t(foot)],
          previousFoot[size_t(foot)],previousFootTime[size_t(foot)],planted[size_t(foot)],
          result.plantedFootFrames);
    frame=syncFaceHead(frame);
    if(!writeFrame(output,frame)){result.error=output.errorString();output.cancelWriting();return result;}
  }
  if(!output.commit()){result.error=output.errorString();return result;}
  if(loop)result.frames-=overlap;
  if(applyInPlace&&result.plantedFootFrames==0) {
    if(!result.warning.isEmpty())result.warning+=' ';
    result.warning+="No reliable planted feet were observed; foot locking was skipped.";
  }
  if(!opt.bvhOutput.isEmpty()) {
    QFile edited(opt.output);
    if(!edited.open(QIODevice::ReadOnly)){result.error=edited.errorString();return result;}
    MocapBvh bvh;MocapRig rig;
    if(!bvh.start(opt.bvhOutput)){result.error=bvh.error();return result;}
    while(!edited.atEnd()) {
      const QJsonObject frame=readFrame(edited);
      if(frame.isEmpty())break;
      bvh.append(rig.retarget(QJsonDocument(frame).toJson(QJsonDocument::Compact)));
    }
    if(!bvh.stop()){result.error=bvh.error();return result;}
  }
  return result;
}

MocapClipIndex MocapClipIndex::build(const QString& path) {
  MocapClipIndex index;index.source=path;
  QFile file(path);
  if(!file.open(QIODevice::ReadOnly)){index.error=file.errorString();return index;}
  index.compact=file.peek(4)=="RMC1";
  file.setProperty("compact",index.compact);
  if(index.compact)file.read(4);
  while(!file.atEnd()) {
    const qint64 offset=file.pos();
    const QJsonObject frame=readFrame(file);
    if(frame.isEmpty())break;
    if(index.frames%30==0)index.anchors.push_back(offset);
    const double t=frame.value("t").toDouble();
    if(index.frames==0)index.firstTime=t;
    index.lastTime=t;++index.frames;
  }
  if(file.property("decodeError").isValid())index.error=file.property("decodeError").toString();
  if(index.frames==0&&index.error.isEmpty())index.error="Clip has no readable frames";
  return index;
}

QJsonObject MocapClipIndex::frameAt(QFile& file,int index) const {
  if(index<0||index>=frames||anchors.empty())return {};
  const int block=index/30;
  if(!file.seek(anchors[size_t(block)]))return {};
  file.setProperty("compact",compact);
  QJsonObject frame;
  for(int i=block*30;i<=index;++i) {
    frame=readFrame(file);if(frame.isEmpty())return {};
  }
  return frame;
}
