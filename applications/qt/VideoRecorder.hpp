#pragma once

#include <QImage>
#include <QString>
#include <memory>

// Windows/Linux recording uses an installed or bundled FFmpeg executable.
class VideoRecorder {
 public:
  VideoRecorder();
  ~VideoRecorder();
  VideoRecorder(const VideoRecorder&) = delete;
  VideoRecorder& operator=(const VideoRecorder&) = delete;
  bool start(const QString& path, int width, int height);
  bool append(const QImage& frame);
  bool stop();
  bool active() const;
  QString error() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
