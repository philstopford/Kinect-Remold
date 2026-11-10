#pragma once

#include <QByteArray>
#include <QJsonObject>
#include <QQuaternion>
#include <QVector3D>
#include <QString>
#include <array>

struct HumanoidBone {
  QVector3D position;
  QVector3D localPosition;
  QQuaternion rotation;
  QQuaternion localRotation;
  float confidence=0;
};

struct HumanoidPose {
  static constexpr int count=19;
  std::array<HumanoidBone,count> bones{};
  double time=0;
  int frame=0;
  bool tracked=false;
};

class MocapRig {
 public:
  HumanoidPose retarget(const QByteArray& cameraJson,bool mirror=false);
  static QJsonObject unityJson(const QByteArray& cameraJson,const HumanoidPose& pose,bool mirrored=false);
  static const char* name(int bone);
  static int parent(int bone);
  static QVector3D restOffset(int bone);
 private:
  QVector3D origin_;
  bool hasOrigin_=false;
  float floor_=0;
  bool hasFloor_=false;
  int missed_=0;
  std::array<float,19> lengths_{};
  std::array<int,19> lengthSamples_{};
  std::array<QQuaternion,19> previousLocal_{};
  std::array<QVector3D,19> previousPosition_{};
  bool hasPrevious_=false;
  float headRollDegrees_=0;
  bool hasHeadRoll_=false;
  double previousTime_=0;
  QString previousDevice_;
  int previousTrackingId_=0;
  bool previousMirror_=false;
};
