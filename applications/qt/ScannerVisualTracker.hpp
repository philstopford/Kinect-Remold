#pragma once

#include <QByteArray>
#include <QImage>
#include "DepthCalibration.hpp"
#include "TsdfVolume.hpp"

struct ScannerVisualResult {
  ScannerPose currentToReference;
  int matches=0;
  int inliers=0;
  float rmsM=0;
  bool accepted=false;
};

// Matches registered RGB detail with metric depth so flat objects can provide
// lateral camera motion that point-to-plane depth ICP cannot observe.
class ScannerVisualTracker {
 public:
  ScannerVisualResult align(const QByteArray& currentDepth,const QByteArray& referenceDepth,
      const DepthIntrinsics& camera,const QImage& currentColor,const QImage& referenceColor) const;
};
