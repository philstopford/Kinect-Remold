#pragma once

#include <QByteArray>
#include <QSaveFile>
#include <QString>
#include <memory>
#include <vector>

// Indexed AVI 1.0 MJPEG writer used by surveillance, including retained pre-roll.
class MjpegAviWriter {
 public:
  MjpegAviWriter() = default;
  bool start(const QString& path, int width, int height, int fps);
  bool append(const QByteArray& jpeg);
  bool finish();
  bool active() const { return bool(file_); }
  QString error() const { return error_; }
 private:
  struct Index { quint32 offset = 0, bytes = 0; };
  bool fourcc(const char* code);
  bool u16(quint16 value);
  bool u32(quint32 value);
  bool patch(quint64 offset, quint32 value);
  bool list(const char* type, quint64& sizeOffset);
  bool endList(quint64 sizeOffset);
  std::unique_ptr<QSaveFile> file_;
  std::vector<Index> index_;
  QString error_;
  quint64 riffSize_ = 0, avihFrames_ = 0, avihMaxBytes_ = 0;
  quint64 strhFrames_ = 0, moviSize_ = 0, moviStart_ = 0;
  quint64 payloadBytes_ = 0;
  quint32 maxFrameBytes_ = 0;
  int width_ = 0, height_ = 0, fps_ = 0;
};
