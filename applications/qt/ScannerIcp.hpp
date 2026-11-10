#pragma once

#include <QByteArray>
#include "DepthCalibration.hpp"
#include "TsdfVolume.hpp"

struct ScannerIcpResult {
  ScannerPose currentToReference;
  float rmsM = 0;
  float overlap = 0;
  int matches = 0;
  bool accepted = false;
};

class ScannerIcp {
 public:
  ScannerIcpResult align(const QByteArray& current, const QByteArray& reference,
                         const DepthIntrinsics& camera,
                         const DepthCalibrationProfile* calibration,
                         bool turntable = false) const;
};
