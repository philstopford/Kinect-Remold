#pragma once

#include <QImage>
#include <vector>

struct MotionSample {
  float score = 0;
  float meanLuma = 0;
  bool moving = false;
  bool triggered = false;
};

// Temporal luma detector shared by RGB and infrared surveillance streams.
class MotionDetector {
 public:
  void reset(int warmupFrames = 12);
  MotionSample process(const QImage& image);
 private:
  std::vector<float> background_;
  int warmup_ = 12;
  int consecutive_ = 0;
  bool moving_ = false;
};
