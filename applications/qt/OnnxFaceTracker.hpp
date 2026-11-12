#pragma once

#include "macos/OnnxBodyTracker.hpp"
#include <QImage>
#include <QJsonObject>
#include <QString>
#include <array>
#include <memory>

// MediaPipe's 478-point face landmark model converted to ONNX. Coordinates in
// the returned JSON are normalized to the source RGB image, not the crop.
class OnnxFaceTracker {
 public:
  explicit OnnxFaceTracker(const QString& modelPath);
  ~OnnxFaceTracker();
  bool ready() const;
  QString error() const;
  QString provider() const;
  QJsonObject track(const QImage& image,const std::array<BodyKeypoint,17>& body,
                    QImage* annotated=nullptr);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
