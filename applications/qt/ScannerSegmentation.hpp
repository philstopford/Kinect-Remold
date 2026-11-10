#pragma once

#include <QByteArray>
#include <QVector3D>
#include "DepthCalibration.hpp"

struct ScannerSupportPlane {
  QVector3D normal;
  float offset = 0;
  int inliers = 0;
  float coverage = 0;
  bool valid = false;
  void removeSupport(QByteArray& metricMm16, const DepthIntrinsics& camera,
                     const DepthCalibrationProfile* calibration) const;
};

class ScannerSegmentation {
 public:
  static ScannerSupportPlane findSupport(const QByteArray& metricMm16,
      const DepthIntrinsics& camera,const DepthCalibrationProfile* calibration);
};
