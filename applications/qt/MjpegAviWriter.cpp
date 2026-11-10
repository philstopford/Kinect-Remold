#include "MjpegAviWriter.hpp"

#include <algorithm>

bool MjpegAviWriter::fourcc(const char* code) { return file_->write(code, 4) == 4; }
bool MjpegAviWriter::u16(quint16 value) {
  const char bytes[]{char(value), char(value >> 8)};
  return file_->write(bytes, 2) == 2;
}
bool MjpegAviWriter::u32(quint32 value) {
  const char bytes[]{char(value), char(value >> 8), char(value >> 16), char(value >> 24)};
  return file_->write(bytes, 4) == 4;
}
bool MjpegAviWriter::patch(quint64 offset, quint32 value) {
  const qint64 current = file_->pos();
  return file_->seek(qint64(offset)) && u32(value) && file_->seek(current);
}
bool MjpegAviWriter::list(const char* type, quint64& sizeOffset) {
  if (!fourcc("LIST")) return false;
  sizeOffset = quint64(file_->pos());
  return u32(0) && fourcc(type);
}
bool MjpegAviWriter::endList(quint64 sizeOffset) {
  const quint64 size = quint64(file_->pos()) - (sizeOffset + 4);
  return size <= 0xffffffffu && patch(sizeOffset, quint32(size));
}

bool MjpegAviWriter::start(const QString& path, int width, int height, int fps) {
  if (active() && !finish()) return false;
  error_.clear(); index_.clear(); payloadBytes_ = 0; maxFrameBytes_ = 0;
  if (width <= 0 || height <= 0 || width > 4096 || height > 4096 || fps < 1 || fps > 60) {
    error_ = QStringLiteral("Invalid AVI video dimensions or frame rate"); return false;
  }
  width_ = width; height_ = height; fps_ = fps;
  file_ = std::make_unique<QSaveFile>(path);
  if (!file_->open(QIODevice::WriteOnly)) { error_ = file_->errorString(); file_.reset(); return false; }
  quint64 hdrl = 0, strl = 0;
  bool ok = fourcc("RIFF"); riffSize_ = quint64(file_->pos());
  ok = ok && u32(0) && fourcc("AVI ") && list("hdrl", hdrl);
  ok = ok && fourcc("avih") && u32(56) && u32(quint32(1000000 / fps_));
  avihMaxBytes_ = quint64(file_->pos());
  ok = ok && u32(0) && u32(0) && u32(0x10);
  avihFrames_ = quint64(file_->pos());
  ok = ok && u32(0) && u32(0) && u32(1) && u32(0) && u32(quint32(width_)) && u32(quint32(height_)) &&
      u32(0) && u32(0) && u32(0) && u32(0);
  ok = ok && list("strl", strl) && fourcc("strh") && u32(56) && fourcc("vids") && fourcc("MJPG") &&
      u32(0) && u16(0) && u16(0) && u32(0) && u32(1) && u32(quint32(fps_)) && u32(0);
  strhFrames_ = quint64(file_->pos());
  ok = ok && u32(0) && u32(0) && u32(0xffffffffu) && u32(0) && u16(0) && u16(0) &&
      u16(quint16(width_)) && u16(quint16(height_));
  ok = ok && fourcc("strf") && u32(40) && u32(40) && u32(quint32(width_)) &&
      u32(quint32(height_)) && u16(1) && u16(24) && fourcc("MJPG") &&
      u32(quint32(width_ * height_ * 3)) && u32(0) && u32(0) && u32(0) && u32(0);
  ok = ok && endList(strl) && endList(hdrl) && fourcc("LIST");
  moviSize_ = quint64(file_->pos());
  ok = ok && u32(0);
  moviStart_ = quint64(file_->pos());
  ok = ok && fourcc("movi");
  if (!ok) { error_ = file_->errorString(); file_.reset(); return false; }
  return true;
}

bool MjpegAviWriter::append(const QByteArray& jpeg) {
  if (!active()) { error_ = QStringLiteral("AVI writer is closed"); return false; }
  if (jpeg.size() < 4 || !jpeg.startsWith("\xff\xd8") || !jpeg.endsWith("\xff\xd9")) {
    error_ = QStringLiteral("Invalid JPEG frame"); return false;
  }
  const quint64 offset = quint64(file_->pos());
  if (offset > 0xffffffffu || jpeg.size() > 0xffffffffu) {
    error_ = QStringLiteral("AVI 1.0 size limit reached"); return false;
  }
  if (!fourcc("00dc") || !u32(quint32(jpeg.size())) || file_->write(jpeg) != jpeg.size() ||
      ((jpeg.size() & 1) && file_->write("\0", 1) != 1)) {
    error_ = file_->errorString(); return false;
  }
  index_.push_back({quint32(offset - moviStart_), quint32(jpeg.size())});
  maxFrameBytes_ = std::max(maxFrameBytes_, quint32(jpeg.size()));
  payloadBytes_ += quint64(jpeg.size());
  return true;
}

bool MjpegAviWriter::finish() {
  if (!active()) return error_.isEmpty();
  if (index_.empty()) {
    error_ = QStringLiteral("No video frames were recorded");
    file_.reset(); return false;
  }
  if (!error_.isEmpty()) { file_.reset(); return false; }
  const quint64 moviEnd = quint64(file_->pos());
  bool ok = moviEnd <= 0xffffffffu && patch(moviSize_, quint32(moviEnd - (moviSize_ + 4))) &&
      index_.size() <= 0x0fffffffu && fourcc("idx1") && u32(quint32(index_.size() * 16));
  for (const Index& frame : index_)
    ok = ok && fourcc("00dc") && u32(0x10) && u32(frame.offset) && u32(frame.bytes);
  const quint64 end = quint64(file_->pos());
  const quint32 bytesPerSecond = index_.empty() ? 0 : quint32(std::min<quint64>(0xffffffffu,
      std::max<quint64>(maxFrameBytes_, payloadBytes_ * quint64(fps_) / index_.size())));
  ok = ok && end >= 8 && end <= 0xffffffffu && patch(avihFrames_, quint32(index_.size())) &&
      patch(strhFrames_, quint32(index_.size())) && patch(avihMaxBytes_, bytesPerSecond) &&
      patch(riffSize_, quint32(end - 8));
  if (ok) ok = file_->commit();
  if (!ok && error_.isEmpty()) error_ = file_->errorString();
  file_.reset();
  return ok;
}
