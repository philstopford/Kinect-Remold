#include "VideoRecorder.hpp"

#include <QCoreApplication>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>

struct VideoRecorder::Impl {
  QProcess process;
  QString error;
  int width = 0;
  int height = 0;
  bool started = false;
};

VideoRecorder::VideoRecorder() : impl_(std::make_unique<Impl>()) {}
VideoRecorder::~VideoRecorder() { stop(); }
bool VideoRecorder::active() const { return impl_->process.state() == QProcess::Running; }
QString VideoRecorder::error() const { return impl_->error; }

bool VideoRecorder::start(const QString& path, int width, int height) {
  stop();
  impl_->error.clear();
  if (width <= 0 || height <= 0 || width > 4096 || height > 4096) {
    impl_->error = QStringLiteral("Invalid video dimensions"); return false;
  }
  const QString bundled = QCoreApplication::applicationDirPath() + "/ffmpeg";
#ifdef Q_OS_WIN
  const QString bundledWindows = bundled + ".exe";
  const QString executable = QFileInfo::exists(bundledWindows) ? bundledWindows :
      QStandardPaths::findExecutable("ffmpeg.exe");
#else
  const QString executable = QFileInfo::exists(bundled) ? bundled :
      QStandardPaths::findExecutable("ffmpeg");
#endif
  if (executable.isEmpty()) {
    impl_->error = QStringLiteral("FFmpeg is required for video recording"); return false;
  }
  impl_->width = width; impl_->height = height;
  impl_->process.setProgram(executable);
  impl_->process.setArguments({"-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
      "-pixel_format", "bgra", "-video_size", QString::number(width) + "x" + QString::number(height),
      "-framerate", "30", "-use_wallclock_as_timestamps", "1", "-i", "pipe:0",
      "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-fps_mode", "vfr", path});
  impl_->process.setProcessChannelMode(QProcess::MergedChannels);
  impl_->process.start();
  if (!impl_->process.waitForStarted(5000)) {
    impl_->error = impl_->process.errorString(); return false;
  }
  impl_->started = true;
  return true;
}

bool VideoRecorder::append(const QImage& frame) {
  if (!active()) {
    impl_->error = QString::fromUtf8(impl_->process.readAll()).trimmed();
    if (impl_->error.isEmpty()) impl_->error = QStringLiteral("Video encoder stopped");
    return false;
  }
  const qint64 frameBytes = qint64(impl_->width) * impl_->height * 4;
  // Keep the GUI responsive when encoding cannot keep up with the camera.
  if (impl_->process.bytesToWrite() > frameBytes)
    impl_->process.waitForBytesWritten(10);
  if (impl_->process.bytesToWrite() > frameBytes * 2) return true;
  const QImage bgra = frame.scaled(impl_->width, impl_->height, Qt::IgnoreAspectRatio,
                                   Qt::FastTransformation).convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < impl_->height; ++y)
    if (impl_->process.write(reinterpret_cast<const char*>(bgra.constScanLine(y)), impl_->width * 4) !=
        impl_->width * 4) {
      impl_->error = impl_->process.errorString(); return false;
    }
  return true;
}

bool VideoRecorder::stop() {
  if (!impl_->started) return impl_->error.isEmpty();
  if (active()) {
    impl_->process.closeWriteChannel();
    if (!impl_->process.waitForFinished(15000)) {
      impl_->process.kill(); impl_->process.waitForFinished(2000);
      impl_->error = QStringLiteral("Timed out finalizing video");
    }
  }
  if (impl_->process.exitStatus() != QProcess::NormalExit || impl_->process.exitCode() != 0) {
    impl_->error = QString::fromUtf8(impl_->process.readAll()).trimmed();
    if (impl_->error.isEmpty()) impl_->error = QStringLiteral("Video encoding failed");
  }
  impl_->started = false;
  return impl_->error.isEmpty();
}
