#pragma once

#include <QByteArray>
#include <QImage>
#include <QString>
#include <array>
#include <memory>

#include "DepthCalibration.hpp"

struct SynSkeletonJoint {
  float x = 0, y = 0, z = 0, confidence = 0;
  bool tracked = false;
};

class SynSkeletonTracker {
 public:
  static constexpr int jointCount = 58;
  explicit SynSkeletonTracker(const QString& modelPath);
  ~SynSkeletonTracker();
  bool ready() const;
  QString error() const;
  bool infer(const QByteArray& metricMm16, const DepthIntrinsics& intrinsics,
             const DepthCalibrationProfile* calibration,
             std::array<SynSkeletonJoint, jointCount>& joints);
  static QImage annotate(const QImage& depthPreview,
                         const std::array<SynSkeletonJoint, jointCount>& joints,
                         const DepthIntrinsics& camera);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
