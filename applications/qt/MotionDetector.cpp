#include "MotionDetector.hpp"

#include <algorithm>
#include <cmath>

void MotionDetector::reset(int warmupFrames) {
  background_.clear();
  warmup_ = warmupFrames;
  consecutive_ = 0;
  moving_ = false;
}

MotionSample MotionDetector::process(const QImage& image) {
  MotionSample result;
  if (image.isNull()) return result;
  constexpr int width = 80, height = 60;
  const QImage gray = image.scaled(width, height, Qt::IgnoreAspectRatio,
                                   Qt::FastTransformation).convertToFormat(QImage::Format_Grayscale8);
  constexpr int count = width * height;
  if (background_.size() != count) {
    background_.assign(count, -1.0f);
    warmup_ = std::max(warmup_, 12);
    consecutive_ = 0;
    moving_ = false;
  }
  std::vector<float> current(count);
  float globalDelta = 0;
  int valid = 0;
  for (int y = 0; y < height; ++y) {
    const uchar* row = gray.constScanLine(y);
    for (int x = 0; x < width; ++x) {
      const int i = y * width + x;
      current[size_t(i)] = row[x] / 255.0f;
      result.meanLuma += current[size_t(i)];
      if (background_[size_t(i)] >= 0) {
        globalDelta += current[size_t(i)] - background_[size_t(i)];
        ++valid;
      }
    }
  }
  result.meanLuma /= count;
  const float illumination = valid ? std::clamp(globalDelta / valid, -0.18f, 0.18f) : 0;
  std::vector<bool> changed(count, false);
  for (int i = 0; i < count; ++i)
    changed[size_t(i)] = background_[size_t(i)] >= 0 &&
        std::abs(current[size_t(i)] - background_[size_t(i)] - illumination) >= 0.10f;
  int coherent = 0;
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
    if (!changed[size_t(y * width + x)]) continue;
    int neighbors = 0;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if ((!dx && !dy) || x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) continue;
      if (changed[size_t((y + dy) * width + x + dx)]) ++neighbors;
    }
    if (neighbors >= 2) ++coherent;
  }
  result.score = valid ? float(coherent) / valid : 0;
  result.moving = valid >= 400 && result.score >= (moving_ ? 0.022f : 0.040f);
  if (warmup_ > 0) {
    --warmup_; result.moving = false; consecutive_ = 0;
  } else {
    consecutive_ = result.moving ? consecutive_ + 1 : 0;
    result.triggered = consecutive_ >= 5;
  }
  moving_ = result.moving;
  const float alpha = result.moving ? 0.003f : 0.028f;
  for (int i = 0; i < count; ++i)
    background_[size_t(i)] = background_[size_t(i)] < 0 ? current[size_t(i)] :
        background_[size_t(i)] + alpha * (current[size_t(i)] - background_[size_t(i)]);
  return result;
}
