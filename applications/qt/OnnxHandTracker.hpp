#pragma once

#include <QImage>
#include <QString>
#include <memory>
#include "macos/OnnxBodyTracker.hpp"

class OnnxHandTracker {
 public:
  explicit OnnxHandTracker(const QString& modelPath);
  ~OnnxHandTracker();
  bool ready() const;
  QString error() const;
  QString provider() const;
  QImage annotate(const QImage& image, const std::array<BodyKeypoint, 17>& body);
  std::array<std::array<BodyKeypoint,21>,2> keypoints() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
