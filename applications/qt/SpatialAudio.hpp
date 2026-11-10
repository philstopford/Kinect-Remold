#pragma once

#include <QByteArray>

struct SpatialAudioResult {
  float azimuthDeg = 0;
  float confidence = 0;
  float rms = 0;
  QByteArray beamPcm16;
};

// GCC-PHAT direction estimate and delay-and-sum beamforming for four microphones.
class SpatialAudio {
 public:
  SpatialAudioResult process(const QByteArray& pcm32, unsigned channelMask);
 private:
  float smoothedAzimuth_ = 0;
  bool haveAzimuth_ = false;
};
