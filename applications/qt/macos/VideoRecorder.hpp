#pragma once
#include <QImage>
#include <QString>
#include <memory>

class VideoRecorder {
 public:
  VideoRecorder();
  ~VideoRecorder();
  bool start(const QString& path, int width, int height);
  bool append(const QImage& frame);
  bool stop();
  bool active() const;
  QString error() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
