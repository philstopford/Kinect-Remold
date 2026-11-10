#pragma once

#include <QByteArray>
#include <QVector3D>
#include <array>
#include "DepthCalibration.hpp"
#include "macos/OnnxBodyTracker.hpp"

struct MetricPoseJoint {
  QVector3D point;
  float confidence = 0;
  bool valid = false;
};

class RgbdRegistration {
 public:
  static constexpr int jointCount = 58;
  static bool projectKinectOne(const DepthIntrinsics& camera,int x,int y,
                               quint16 millimeters,int colorWidth,int colorHeight,
                               float& u,float& v);
  static std::array<MetricPoseJoint,jointCount> lift(
      const QByteArray& metricMm16,const DepthIntrinsics& camera,
      const DepthCalibrationProfile* calibration,int colorWidth,int colorHeight,
      bool kinectOne,const std::array<BodyKeypoint,17>& body,
      const std::array<std::array<BodyKeypoint,21>,2>& hands,
      float expectedPersonDepth=0);
};
