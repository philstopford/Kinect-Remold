#include "OnnxFaceTracker.hpp"

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <QJsonArray>
#include <QLineF>
#include <QFileInfo>
#include <QDir>
#include <QPainter>
#include <QPen>
#include <QMatrix3x3>
#include <QQuaternion>
#include <QVector3D>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
// MediaPipe FaceBlendshapesGraph's HUND landmark subset (Apache-2.0).
constexpr std::array<int,146> subset{
    0,1,4,5,6,7,8,10,13,14,17,21,33,37,39,40,46,52,53,54,55,58,
    61,63,65,66,67,70,78,80,81,82,84,87,88,91,93,95,103,105,107,109,
    127,132,133,136,144,145,146,148,149,150,152,153,154,155,157,158,
    159,160,161,162,163,168,172,173,176,178,181,185,191,195,197,234,
    246,249,251,263,267,269,270,276,282,283,284,285,288,291,293,295,
    296,297,300,308,310,311,312,314,317,318,321,323,324,332,334,336,
    338,356,361,362,365,373,374,375,377,378,379,380,381,382,384,385,
    386,387,388,389,390,397,398,400,402,405,409,415,454,466,468,469,
    470,471,472,473,474,475,476,477};
constexpr const char* coefficientNames[52]{
    "_neutral","browDownLeft","browDownRight","browInnerUp","browOuterUpLeft",
    "browOuterUpRight","cheekPuff","cheekSquintLeft","cheekSquintRight",
    "eyeBlinkLeft","eyeBlinkRight","eyeLookDownLeft","eyeLookDownRight",
    "eyeLookInLeft","eyeLookInRight","eyeLookOutLeft","eyeLookOutRight",
    "eyeLookUpLeft","eyeLookUpRight","eyeSquintLeft","eyeSquintRight",
    "eyeWideLeft","eyeWideRight","jawForward","jawLeft","jawOpen","jawRight",
    "mouthClose","mouthDimpleLeft","mouthDimpleRight","mouthFrownLeft",
    "mouthFrownRight","mouthFunnel","mouthLeft","mouthLowerDownLeft",
    "mouthLowerDownRight","mouthPressLeft","mouthPressRight","mouthPucker",
    "mouthRight","mouthRollLower","mouthRollUpper","mouthShrugLower",
    "mouthShrugUpper","mouthSmileLeft","mouthSmileRight","mouthStretchLeft",
    "mouthStretchRight","mouthUpperUpLeft","mouthUpperUpRight","noseSneerLeft",
    "noseSneerRight"};
}

struct OnnxFaceTracker::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING,"KinectRemoldFace"};
  std::unique_ptr<Ort::Session> session;
  std::unique_ptr<Ort::Session> blendSession;
  std::string inputName,landmarkName,presenceName;
  std::string blendInputName,blendOutputName;
  QString error;
  float cropCx=0,cropCy=0,cropSide=0;
  float cropRoll=0;
  bool hasCrop=false;
  int missed=0;
};

OnnxFaceTracker::OnnxFaceTracker(const QString& path):impl_(std::make_unique<Impl>()) {
  try {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    options.SetIntraOpNumThreads(2);
#ifdef Q_OS_WIN
    const std::wstring nativePath=path.toStdWString();
    impl_->session=std::make_unique<Ort::Session>(impl_->env,nativePath.c_str(),options);
#else
    impl_->session=std::make_unique<Ort::Session>(impl_->env,path.toUtf8().constData(),options);
#endif
    if(impl_->session->GetInputCount()!=1||impl_->session->GetOutputCount()<2)
      throw std::runtime_error("Unexpected face model input/output count");
    Ort::AllocatorWithDefaultOptions allocator;
    impl_->inputName=impl_->session->GetInputNameAllocated(0,allocator).get();
    impl_->landmarkName=impl_->session->GetOutputNameAllocated(0,allocator).get();
    impl_->presenceName=impl_->session->GetOutputNameAllocated(1,allocator).get();
    const auto inputType=impl_->session->GetInputTypeInfo(0);
    const auto input=inputType.GetTensorTypeAndShapeInfo();
    const auto shape=input.GetShape();
    if(input.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||shape.size()!=4||
       (shape[0]!=1&&shape[0]!=-1)||shape[1]!=256||shape[2]!=256||shape[3]!=3)
      {
        std::string detail="Expected face FLOAT [1,256,256,3] RGB input; got type "+
            std::to_string(input.GetElementType())+" shape";
        for(auto dimension:shape)detail+=" "+std::to_string(dimension);
        throw std::runtime_error(detail);
      }
    const auto landmarkType=impl_->session->GetOutputTypeInfo(0);
    const auto presenceType=impl_->session->GetOutputTypeInfo(1);
    const auto landmarks=landmarkType.GetTensorTypeAndShapeInfo();
    const auto presence=presenceType.GetTensorTypeAndShapeInfo();
    if(landmarks.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||
       presence.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      throw std::runtime_error("Expected FLOAT face landmarks and presence logit");
    const QString blendPath=QFileInfo(path).dir().filePath("face-blendshapes.onnx");
#ifdef Q_OS_WIN
    const std::wstring blendNative=blendPath.toStdWString();
    impl_->blendSession=std::make_unique<Ort::Session>(impl_->env,blendNative.c_str(),options);
#else
    impl_->blendSession=std::make_unique<Ort::Session>(impl_->env,blendPath.toUtf8().constData(),options);
#endif
    if(impl_->blendSession->GetInputCount()!=1||impl_->blendSession->GetOutputCount()!=1)
      throw std::runtime_error("Unexpected face blendshape model inputs/outputs");
    impl_->blendInputName=impl_->blendSession->GetInputNameAllocated(0,allocator).get();
    impl_->blendOutputName=impl_->blendSession->GetOutputNameAllocated(0,allocator).get();
    const auto blendInputType=impl_->blendSession->GetInputTypeInfo(0);
    const auto blendOutputType=impl_->blendSession->GetOutputTypeInfo(0);
    const auto blendInput=blendInputType.GetTensorTypeAndShapeInfo();
    const auto blendOutput=blendOutputType.GetTensorTypeAndShapeInfo();
    if(blendInput.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||
       blendOutput.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||
       blendInput.GetShape()!=std::vector<std::int64_t>({1,146,2})||
       blendOutput.GetShape()!=std::vector<std::int64_t>({52}))
      throw std::runtime_error("Expected blendshape FLOAT [1,146,2] to [52]");
  } catch(const std::exception& e) {
    impl_->error=QString::fromUtf8(e.what());impl_->session.reset();
  }
}
OnnxFaceTracker::~OnnxFaceTracker()=default;
bool OnnxFaceTracker::ready() const {return bool(impl_->session);}
QString OnnxFaceTracker::error() const {return impl_->error;}

QJsonObject OnnxFaceTracker::track(const QImage& image,
    const std::array<BodyKeypoint,17>& body,QImage* annotated) {
  if(annotated)*annotated=image;
  if(!ready()||image.isNull())return {};
  const auto nose=body[0],leftEye=body[1],rightEye=body[2];
  const auto leftEar=body[3],rightEar=body[4];
  const auto leftShoulder=body[5],rightShoulder=body[6];
  const float width=float(image.width()),height=float(image.height());
  float side=0;
  if(leftEar.confidence>.35f&&rightEar.confidence>.35f)
    side=std::max(side,std::abs(leftEar.x-rightEar.x)*width*1.42f);
  if(leftEye.confidence>.35f&&rightEye.confidence>.35f)
    side=std::max(side,std::abs(leftEye.x-rightEye.x)*width*3.8f);
  if(leftShoulder.confidence>.35f&&rightShoulder.confidence>.35f)
    side=std::max(side,std::abs(leftShoulder.x-rightShoulder.x)*width*.75f);
  const bool bodyCropValid=nose.confidence>=.35f&&side>=45&&side<=width*.85f;
  float cx=nose.x*width;
  const bool singleEar=(leftEar.confidence>.35f)!=(rightEar.confidence>.35f);
  if(leftEar.confidence>.35f&&rightEar.confidence<=.35f)
    cx=cx*.55f+leftEar.x*width*.45f;
  else if(rightEar.confidence>.35f&&leftEar.confidence<=.35f)
    cx=cx*.55f+rightEar.x*width*.45f;
  // In profile the shoulder estimate is deliberately broad. Tighten the
  // square around nose and visible ear so the face mesh fills the model crop.
  if(singleEar)side*=.72f;
  const float cy=nose.y*height+side*(singleEar?.02f:.12f);
  const float bodySide=side;
  try {
    const QImage rgb=image.convertToFormat(QImage::Format_RGB888);
    std::vector<float> pixels(256*256*3);
    const std::array<std::int64_t,4> shape{1,256,256,3};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    std::array<float,1434> measured{};
    float confidence=0,x0=0,y0=0,usedRoll=0;
    auto candidate=[&](float centerX,float centerY,float trialSide,float roll) {
      // MoveNet's nose/ear/shoulder estimate can contract abruptly during a
      // head tilt. A smaller fallback crop then becomes the next tracking
      // anchor and makes the face mesh shrink for many frames. Keep the
      // current scale until tracking is lost long enough to reset the crop.
      if(impl_->hasCrop)trialSide=std::max(trialSide,impl_->cropSide);
      const float originX=centerX-trialSide*.5f,originY=centerY-trialSide*.5f;
      if(trialSide<45||trialSide>width*.85f||
         originX+trialSide*.75f<0||originX+trialSide*.25f>=width||
         originY+trialSide*.75f<0||originY+trialSide*.25f>=height)return;
      const float cosine=std::cos(roll),sine=std::sin(roll);
      for(int y=0;y<256;++y)for(int x=0;x<256;++x) {
        const float localX=(x+.5f-128.f)*trialSide/256.f;
        const float localY=(y+.5f-128.f)*trialSide/256.f;
        const int sx=std::clamp(int(std::lround(centerX+cosine*localX-sine*localY)),0,rgb.width()-1);
        const int sy=std::clamp(int(std::lround(centerY+sine*localX+cosine*localY)),0,rgb.height()-1);
        const uchar* source=rgb.constScanLine(sy)+sx*3;
        for(int c=0;c<3;++c)pixels[(y*256+x)*3+c]=float(source[c])/255.f;
      }
      auto tensor=Ort::Value::CreateTensor<float>(memory,pixels.data(),pixels.size(),
                                                 shape.data(),shape.size());
      const char* input=impl_->inputName.c_str();
      const char* outputs[]{impl_->landmarkName.c_str(),impl_->presenceName.c_str()};
      auto values=impl_->session->Run(Ort::RunOptions{nullptr},&input,&tensor,1,outputs,2);
      if(values.size()!=2||!values[0].IsTensor()||!values[1].IsTensor()||
         values[0].GetTensorTypeAndShapeInfo().GetElementCount()!=1434||
         values[1].GetTensorTypeAndShapeInfo().GetElementCount()!=1)
        throw std::runtime_error("Face model returned an invalid tensor");
      const float logit=values[1].GetTensorData<float>()[0];
      const float score=1.f/(1.f+std::exp(-logit));
      if(std::isfinite(score)&&score>confidence) {
        confidence=score;x0=originX;y0=originY;side=trialSide;usedRoll=roll;
        std::copy_n(values[0].GetTensorData<float>(),1434,measured.begin());
      }
    };
    if(impl_->hasCrop) {
      candidate(impl_->cropCx,impl_->cropCy,impl_->cropSide,impl_->cropRoll);
      // Body keypoints can disappear in profile. Search near the last good
      // crop before falling back to the current body estimate.
      if(confidence<.55f) {
        const float delta=impl_->cropSide*.12f;
        candidate(impl_->cropCx+delta,impl_->cropCy,impl_->cropSide,impl_->cropRoll);
        candidate(impl_->cropCx-delta,impl_->cropCy,impl_->cropSide,impl_->cropRoll);
        candidate(impl_->cropCx,impl_->cropCy+delta,impl_->cropSide,impl_->cropRoll);
        candidate(impl_->cropCx,impl_->cropCy-delta,impl_->cropSide,impl_->cropRoll);
      }
    }
    if(confidence<.55f&&bodyCropValid) {
      candidate(cx,cy,bodySide,impl_->cropRoll);
      if(confidence<.55f) {
        candidate(cx+bodySide*.1f,cy,bodySide,impl_->cropRoll);
        candidate(cx-bodySide*.1f,cy,bodySide,impl_->cropRoll);
      }
      if(confidence<.55f) {
        candidate(cx,cy-bodySide*.1f,bodySide,impl_->cropRoll);
        candidate(cx,cy+bodySide*.1f,bodySide,impl_->cropRoll);
      }
    }
    if(confidence<.55f) {
      // Dedicated facial capture may show only a close face, leaving MoveNet
      // without usable shoulders or even a nose. Search central face crops.
      const float trial=std::min(width,height)*.8f;
      candidate(width*.5f,height*.5f,trial,0);
      if(confidence<.55f)candidate(width*.5f,height*.5f,trial*.65f,0);
      if(confidence<.55f)candidate(width*.38f,height*.48f,trial,0);
      if(confidence<.55f)candidate(width*.62f,height*.48f,trial,0);
    }
    if(confidence<.55f) {
      if(++impl_->missed>15) {impl_->hasCrop=false;impl_->cropRoll=0;}
      return {};
    }
    const bool hadCrop=impl_->hasCrop;
    const float priorCx=impl_->cropCx,priorCy=impl_->cropCy,priorSide=impl_->cropSide;
    impl_->cropCx=x0+side*.5f;impl_->cropCy=y0+side*.5f;
    impl_->cropSide=side;impl_->hasCrop=true;impl_->missed=0;
    const float mappedCx=impl_->cropCx,mappedCy=impl_->cropCy;
    const float* points=measured.data();
    const float cosine=std::cos(usedRoll),sine=std::sin(usedRoll);
    auto imagePoint=[&](int index) {
      const float localX=(points[index*3]-128.f)*side/256.f;
      const float localY=(points[index*3+1]-128.f)*side/256.f;
      return QPointF(mappedCx+cosine*localX-sine*localY,
          mappedCy+sine*localX+cosine*localY);
    };
    const QPointF eyeLeftPoint=imagePoint(33),eyeRightPoint=imagePoint(263);
    const float measuredRoll=std::clamp(float(std::atan2(
        eyeRightPoint.y()-eyeLeftPoint.y(),eyeRightPoint.x()-eyeLeftPoint.x())),-.9f,.9f);
    if(confidence>.75f&&QLineF(eyeLeftPoint,eyeRightPoint).length()>20&&
       (!hadCrop||std::abs(measuredRoll-impl_->cropRoll)<.45f))
      impl_->cropRoll=hadCrop?impl_->cropRoll*.6f+measuredRoll*.4f:measuredRoll;
    QJsonArray landmarks;
    std::array<float,146*2> blendInputPixels{};
    for(int i=0;i<478;++i) {
      const QPointF mapped=imagePoint(i);
      const float px=float(mapped.x()),py=float(mapped.y());
      const float z=points[i*3+2]*side/256.f;
      if(!std::isfinite(px)||!std::isfinite(py)||!std::isfinite(z)||
         std::abs(z)>width*2)return {};
      landmarks.append(QJsonArray{px/width,py/height,z/width});
    }
    // Keep the face away from the crop boundary. A reacquisition after a
    // missed turn can select a tight body crop; the model then extrapolates
    // landmarks near its edges and the filled mesh visibly stretches.
    static constexpr int oval[]{10,338,297,332,284,251,389,356,454,323,361,288,
        397,365,379,378,400,377,152,148,176,149,150,136,172,58,132,93,234,
        127,162,21,54,103,67,109};
    float minX=width,maxX=0,minY=height,maxY=0;
    for(int index:oval) {
      const QPointF p=imagePoint(index);
      minX=std::min(minX,float(p.x()));maxX=std::max(maxX,float(p.x()));
      minY=std::min(minY,float(p.y()));maxY=std::max(maxY,float(p.y()));
    }
    const float faceSpan=std::max(maxX-minX,maxY-minY);
    const float cropOccupancy=faceSpan/side;
    if(confidence>.7f&&std::isfinite(cropOccupancy)&&cropOccupancy>.7f) {
      impl_->cropSide=std::min(width*.85f,std::max(side,faceSpan/.65f));
      impl_->cropCx=impl_->cropCx*.75f+(minX+maxX)*.125f;
      impl_->cropCy=impl_->cropCy*.75f+(minY+maxY)*.125f;
    }
    for(size_t i=0;i<subset.size();++i) {
      const int index=subset[i];
      const QPointF mapped=imagePoint(index);
      blendInputPixels[i*2]=float(mapped.x());
      blendInputPixels[i*2+1]=float(mapped.y());
    }
    const std::array<std::int64_t,3> blendShape{1,146,2};
    auto blendTensor=Ort::Value::CreateTensor<float>(memory,blendInputPixels.data(),
        blendInputPixels.size(),blendShape.data(),blendShape.size());
    const char* blendInputName=impl_->blendInputName.c_str();
    const char* blendOutputName=impl_->blendOutputName.c_str();
    auto blendValues=impl_->blendSession->Run(Ort::RunOptions{nullptr},
        &blendInputName,&blendTensor,1,&blendOutputName,1);
    if(blendValues.size()!=1||!blendValues[0].IsTensor()||
       blendValues[0].GetTensorTypeAndShapeInfo().GetElementCount()!=52)
      throw std::runtime_error("Face blendshape model returned an invalid tensor");
    const float* weights=blendValues[0].GetTensorData<float>();
    QJsonObject blendshapes;
    for(int i=1;i<52;++i) {
      if(!std::isfinite(weights[i]))throw std::runtime_error("Non-finite face blendshape");
      blendshapes[QLatin1String(coefficientNames[i])]=std::clamp(weights[i],0.f,1.f);
    }
    auto score=[&](const char* name){return blendshapes.value(QLatin1String(name)).toDouble();};
    const QJsonObject eyeGaze{{"left",QJsonArray{
        score("eyeLookOutLeft")-score("eyeLookInLeft"),
        score("eyeLookUpLeft")-score("eyeLookDownLeft")}},
        {"right",QJsonArray{
        score("eyeLookInRight")-score("eyeLookOutRight"),
        score("eyeLookUpRight")-score("eyeLookDownRight")}},
        {"units","score-difference"}};
    auto facePoint=[&](int index) {
      const QPointF mapped=imagePoint(index);
      return QVector3D(float(mapped.x()),float(mapped.y()),
          points[index*3+2]*side/256.f);
    };
    QVector3D right=facePoint(263)-facePoint(33);
    QVector3D up=facePoint(10)-facePoint(152);
    QJsonObject head;
    if(right.lengthSquared()>1&&up.lengthSquared()>1) {
      right.normalize();up-=right*QVector3D::dotProduct(up,right);
      if(up.lengthSquared()>1) {
        up.normalize();
        const QVector3D front=QVector3D::crossProduct(right,up).normalized();
        QMatrix3x3 m;
        m(0,0)=right.x();m(1,0)=right.y();m(2,0)=right.z();
        m(0,1)=-up.x();m(1,1)=-up.y();m(2,1)=-up.z();
        m(0,2)=-front.x();m(1,2)=-front.y();m(2,2)=-front.z();
        const QQuaternion q=QQuaternion::fromRotationMatrix(m).normalized();
        head["q"]=QJsonArray{q.x(),q.y(),q.z(),q.scalar()};
        const QPointF center=imagePoint(1);
        head["uv"]=QJsonArray{center.x()/width,center.y()/height};
      }
    }
    if(annotated) {
      *annotated=image.copy();
      QPainter painter(annotated);painter.setRenderHint(QPainter::Antialiasing);
      painter.setPen(QPen(QColor(246,105,192),2));
      painter.setBrush(QColor(246,105,192));
      for(int i=0;i<478;i+=4)
        painter.drawEllipse(imagePoint(i),2,2);
    }
    impl_->error.clear();
    float yawDegrees=0;
    if(head.contains("q")) {
      const QJsonArray rotation=head.value("q").toArray();
      const QQuaternion q(float(rotation[3].toDouble()),float(rotation[0].toDouble()),
          float(rotation[1].toDouble()),float(rotation[2].toDouble()));
      const QVector3D forward=q.rotatedVector(QVector3D(0,0,1));
      yawDegrees=float(std::atan2(forward.x(),forward.z())*57.2957795);
    }
    const float rollDegrees=measuredRoll*57.2957795f;
    const bool unusable=!head.contains("q")||confidence<.65f||cropOccupancy>.9f||
        std::abs(yawDegrees)>50.f||std::abs(rollDegrees)>45.f;
    const bool marginal=!unusable&&(confidence<.85f||cropOccupancy>.72f||
        std::abs(yawDegrees)>32.f||std::abs(rollDegrees)>28.f);
    const QString reason=cropOccupancy>.72f?"adjust-face-distance":
        std::abs(yawDegrees)>32.f?"turn-toward-camera":
        std::abs(rollDegrees)>28.f?"reduce-head-tilt":
        confidence<.85f?"weak-landmarks":"face-visible";
    QJsonObject result{{"tracked",!unusable},{"confidence",confidence},
        {"model","mediapipe-face-landmarks-478"},{"landmarks",landmarks},
        {"blendshapes",blendshapes},{"head",head},{"eyeGaze",eyeGaze},
        {"quality",QJsonObject{{"state",unusable?"red":marginal?"yellow":"green"},
            {"reason",reason},{"yawDegrees",yawDegrees},{"rollDegrees",rollDegrees}}}};
    if(!qEnvironmentVariableIsEmpty("REMOLD_SOLVE_CAPTURE"))
      result["cropDebug"]=QJsonObject{{"x",x0},{"y",y0},{"side",side},
          {"priorCx",priorCx},{"priorCy",priorCy},{"priorSide",priorSide},
          {"usedRoll",usedRoll},{"nextRoll",impl_->cropRoll},
          {"hadCrop",hadCrop},{"bodyCx",cx},{"bodyCy",cy},
          {"bodySide",bodySide},{"bodyCropValid",bodyCropValid},
          {"cropOccupancy",cropOccupancy},
          {"imageWidth",width},{"imageHeight",height}};
    return result;
  } catch(const std::exception& e) {
    impl_->error=QString::fromUtf8(e.what());return {};
  }
}
