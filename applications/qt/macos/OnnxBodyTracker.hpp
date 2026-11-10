#pragma once

#include <QImage>
#include <QString>
#include <memory>
#include <array>

struct BodyKeypoint { float x = 0, y = 0, confidence = 0; };

// MoveNet model input is checked at runtime against the verified bundled asset.
class OnnxBodyTracker {
 public:
  explicit OnnxBodyTracker(const QString& modelPath);
  ~OnnxBodyTracker();
  OnnxBodyTracker(const OnnxBodyTracker&) = delete;
  OnnxBodyTracker& operator=(const OnnxBodyTracker&) = delete;
  bool ready() const;
  QString error() const;
  QImage annotate(const QImage& image);
  std::array<BodyKeypoint, 17> keypoints() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
