#include <QApplication>
#include <QBuffer>
#include <QCheckBox>
#include <QCoreApplication>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QLocalSocket>
#include <QMainWindow>
#include <QMouseEvent>
#include <QMutex>
#include <QMutexLocker>
#include <QPaintEvent>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QProcess>
#include <QQuaternion>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStorageInfo>
#include <QTemporaryDir>
#include <QTableWidget>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWaitCondition>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>
#ifndef Q_OS_WIN
#include <unistd.h>
#endif
#ifdef Q_OS_MACOS
#include <sys/stat.h>
#endif

#include "remold/protocol.hpp"
#include "KinectOneRemoldProtocol.h"
#include "MotionDetector.hpp"
#include "SpatialAudio.hpp"
#include "MjpegAviWriter.hpp"
#include "DepthCalibration.hpp"
#include "TsdfVolume.hpp"
#include "ScannerIcp.hpp"
#include "ScannerMeshWriter.hpp"
#include "ScannerSegmentation.hpp"
#include "MocapNetwork.hpp"
#include "MocapWebSocket.hpp"
#include "MocapRig.hpp"
#include "FaceMeshTopology.hpp"
#include "MocapBvh.hpp"
#include "MocapClipProcessor.hpp"
#include "LanPoseLink.hpp"
#ifdef Q_OS_MACOS
#include "macos/VisionTracker.hpp"
#include "macos/VideoRecorder.hpp"
#else
#include "VideoRecorder.hpp"
#endif
#ifdef REMOLD_HAS_ONNX
#include "macos/OnnxBodyTracker.hpp"
#include "OnnxHandTracker.hpp"
#include "OnnxFaceTracker.hpp"
#include "SynSkeletonTracker.hpp"
#include "RgbdRegistration.hpp"
#include "ArticulatedPoseFit.hpp"
#endif

namespace {

struct Device {
  QString id, label, generation, state, control, camera, audio, audioControl, sdk;
};

QString calibrationPath(const QString& deviceId);

QString sdkPath(bool one) {
#ifdef Q_OS_WIN
  return one ? QStringLiteral("KinectOneRemoldSdk") : QStringLiteral("Kinect360RemoldSdk");
#elif defined(Q_OS_MACOS)
  const QByteArray key = one ? "REMOLD_MACOS_RUNTIME_ONE" : "REMOLD_MACOS_RUNTIME360";
  const QByteArray configured = qgetenv(key.constData());
  const QString root = configured.isEmpty()
      ? QStringLiteral("/tmp/%1-%2").arg(one ? "kone-remold" : "k360-remold").arg(getuid())
      : QString::fromLocal8Bit(configured);
  return root + QStringLiteral("/sdk.sock");
#else
  return one ? QStringLiteral("/run/kinect-one-remold/sdk.sock")
             : QStringLiteral("/run/kinect360-remold/sdk.sock");
#endif
}

#ifdef Q_OS_MACOS
class BundledUsbHost : public QProcess {
 public:
  ~BundledUsbHost() override {
    if(state()==QProcess::NotRunning)return;
    terminate();
    if(!waitForFinished(2000)) {kill();waitForFinished(1000);}
  }
};

bool startBundledUsbHost(QProcess& host) {
  auto available=[](const QString& path) {
    QLocalSocket socket;socket.connectToServer(path);
    return socket.waitForConnected(100);
  };
  if(available(sdkPath(false))&&available(sdkPath(true)))return true;
  const QString helper=QCoreApplication::applicationDirPath()+"/kinect-remold-native-usb";
  if(!QFileInfo::exists(helper)) {
    qWarning("Bundled native USB host missing: %s",qPrintable(helper));return false;
  }
  for(const auto& runtime:std::array<std::pair<const char*,QString>,2>{{
      {"REMOLD_MACOS_RUNTIME360",QFileInfo(sdkPath(false)).absolutePath()},
      {"REMOLD_MACOS_RUNTIME_ONE",QFileInfo(sdkPath(true)).absolutePath()}}}) {
    const QFileInfo info(runtime.second);
    if(info.isSymLink()||(info.exists()&&!info.isDir())||
       !QDir().mkpath(runtime.second)||
       chmod(QFile::encodeName(runtime.second).constData(),0700)!=0) {
      qWarning("Unsafe native USB runtime directory: %s",qPrintable(runtime.second));
      return false;
    }
    qputenv(runtime.first,QFile::encodeName(runtime.second));
  }
  host.setProgram(helper);host.start();
  if(!host.waitForStarted(2000)) {
    qWarning("Native USB host failed to start: %s",qPrintable(host.errorString()));
    return false;
  }
  QElapsedTimer timer;timer.start();
  while(timer.elapsed()<3000&&host.state()==QProcess::Running) {
    if(available(sdkPath(false))&&available(sdkPath(true)))return true;
    QThread::msleep(50);
  }
  qWarning("Native USB host did not become ready: %s",qPrintable(QString::fromLocal8Bit(host.readAllStandardError())));
  return false;
}
#endif

QByteArray readExact(QLocalSocket& socket, qint64 size, QThread* thread) {
  QByteArray result;
  result.reserve(static_cast<qsizetype>(size));
  while (result.size() < size && !thread->isInterruptionRequested()) {
    if (socket.bytesAvailable() == 0 && !socket.waitForReadyRead(500)) {
      if (socket.state() != QLocalSocket::ConnectedState) break;
      continue;
    }
    result += socket.read(size - result.size());
  }
  return result.size() == size ? result : QByteArray{};
}

std::vector<Device> discoverAt(const QString& path) {
  QLocalSocket socket;
  socket.connectToServer(path);
  if (!socket.waitForConnected(150)) return {};
  socket.write("LIST\n", 5);
  if (!socket.waitForBytesWritten(150)) return {};
  QByteArray data;
  while (socket.waitForReadyRead(150)) data += socket.readAll();
  data += socket.readAll();
  std::vector<Device> found;
  for (const QByteArray& line : data.split('\n')) {
    const QList<QByteArray> fields = line.split('\t');
    if (fields.size() < 11 || fields[0].isEmpty()) continue;
    Device d;
    d.id = QString::fromUtf8(fields[0]); d.label = QString::fromUtf8(fields[1]);
    d.state = QString::fromUtf8(fields[2]); d.control = QString::fromUtf8(fields[3]);
    d.camera = QString::fromUtf8(fields[4]);
    d.audio = QString::fromUtf8(fields[5]); d.audioControl = QString::fromUtf8(fields[6]);
    d.sdk = QString::fromUtf8(fields[8]); d.generation = QString::fromUtf8(fields[10]);
    found.push_back(std::move(d));
  }
  return found;
}

QString probeAt(const QString& path, const QString& id) {
  QLocalSocket socket;
  socket.connectToServer(path);
  if (!socket.waitForConnected(1000)) return socket.errorString();
  socket.write("PROBE " + id.toUtf8() + "\n");
  if (!socket.waitForBytesWritten(1000) || !socket.waitForReadyRead(2000)) return "Native USB probe timed out";
  return QString::fromUtf8(socket.readAll()).trimmed();
}

QString controlAt(const QString& path, const QString& id, const QByteArray& action) {
  QLocalSocket socket;
  socket.connectToServer(path);
  if (!socket.waitForConnected(1000)) return socket.errorString();
  const QByteArray request = "CONTROL\t" + id.toUtf8() + "\t" + action + "\n";
  if (socket.write(request) != request.size() || !socket.waitForBytesWritten(1000) ||
      !socket.waitForReadyRead(2000)) return QStringLiteral("Control request timed out");
  return QString::fromUtf8(socket.readAll()).trimmed();
}

QString tiltAt(const Device& device, int degrees) {
  if (device.control.isEmpty()) return QStringLiteral("Motor control is unavailable");
  QLocalSocket socket;
  socket.connectToServer(device.control);
  if (!socket.waitForConnected(1500)) return socket.errorString();
  remold::control::Request request{};
  request.command = remold::control::Command::Tilt;
  request.value = degrees;
  const QByteArray id = device.id.toUtf8();
  if (id.size() >= int(sizeof(request.deviceId))) return QStringLiteral("Invalid device ID");
  std::memcpy(request.deviceId, id.constData(), size_t(id.size()));
  if (socket.write(reinterpret_cast<const char*>(&request), sizeof(request)) != sizeof(request) ||
      !socket.waitForBytesWritten(1500)) return QStringLiteral("Motor request failed");
  QByteArray response;
  QElapsedTimer timeout; timeout.start();
  while (response.size() < int(sizeof(remold::control::Reply)) && timeout.elapsed() < 7000) {
    if (socket.bytesAvailable() || socket.waitForReadyRead(500)) response += socket.readAll();
    else if (socket.state() != QLocalSocket::ConnectedState) break;
  }
  if (response.size() != int(sizeof(remold::control::Reply))) return QStringLiteral("Motor response timed out");
  remold::control::Reply reply{};
  std::memcpy(&reply, response.constData(), sizeof(reply));
  if (reply.magic != remold::control::kMagic || reply.version != remold::control::kVersion)
    return QStringLiteral("Invalid motor response");
  if (reply.result != 0) return QStringLiteral("Motor error %1").arg(reply.result);
  return QStringLiteral("Tilt set to %1°").arg(degrees);
}

QString audioControlAt(const Device& device, remold::audio_control::Command command, int value) {
  if (device.audioControl.isEmpty()) return QStringLiteral("Audio control is unavailable");
  QLocalSocket socket;
  socket.connectToServer(device.audioControl);
  if (!socket.waitForConnected(1000)) return socket.errorString();
  remold::audio_control::Request request{};
  request.command = command; request.value = value;
  if (socket.write(reinterpret_cast<const char*>(&request), sizeof(request)) != sizeof(request) ||
      !socket.waitForBytesWritten(1000)) return QStringLiteral("Audio control request failed");
  QByteArray response;
  QElapsedTimer timeout; timeout.start();
  while (response.size() < int(sizeof(remold::audio_control::Reply)) && timeout.elapsed() < 2000) {
    if (socket.bytesAvailable() || socket.waitForReadyRead(250)) response += socket.readAll();
    else if (socket.state() != QLocalSocket::ConnectedState) break;
  }
  if (response.size() != int(sizeof(remold::audio_control::Reply)))
    return QStringLiteral("Audio control response timed out");
  remold::audio_control::Reply reply{};
  std::memcpy(&reply, response.constData(), sizeof(reply));
  if (reply.magic != remold::audio_control::kMagic || reply.version != remold::audio_control::kVersion)
    return QStringLiteral("Invalid audio control response");
  if (reply.result != 0) return QStringLiteral("Audio control error %1").arg(reply.result);
  return QStringLiteral("Volume %1% · %2").arg(reply.volume).arg(reply.muted ? "muted" : "live");
}

quint32 u32(const char* p) { quint32 value; std::memcpy(&value, p, sizeof(value)); return value; }
quint64 u64(const char* p) { quint64 value; std::memcpy(&value, p, sizeof(value)); return value; }
quint16 u16(const char* p) { quint16 value; std::memcpy(&value, p, sizeof(value)); return value; }
double f64(const char* p) { double value; std::memcpy(&value, p, sizeof(value)); return value; }

QImage grayPacked(const QByteArray& data, int width, int height, int bits, bool metric) {
  if (width <= 0 || height <= 0 || width > 2048 || height > 1536) return {};
  const qsizetype pixels = qsizetype(width) * height;
  if (metric ? data.size() < pixels * 2 : data.size() * 8 < pixels * bits) return {};
  QImage image(width, height, QImage::Format_Grayscale8);
  const auto* src = reinterpret_cast<const quint8*>(data.constData());
  for (qsizetype i = 0; i < pixels; ++i) {
    quint32 sample;
    if (bits == 16) sample = u16(data.constData() + i * 2);
    else {
      // The Xbox 360 camera packs samples most significant bit first.
      const qsizetype bit = i * bits;
      sample = 0;
      for (int b = 0; b < bits; ++b)
        sample = (sample << 1) | ((src[(bit + b) / 8] >> (7 - (bit + b) % 8)) & 1u);
    }
    quint8 value = 0;
    if (metric) value = sample ? quint8(std::min(255u, sample * 255u / 4500u)) : 0;
    else if (bits == 16) value = quint8((sample & 0x3ffu) * 255u / 1023u);
    else value = quint8(sample >> std::max(0, bits - 8));
    image.scanLine(int(i / width))[i % width] = value;
  }
  return image;
}

QImage bayerGrbg(const QByteArray& data, int width, int height) {
  if (width <= 0 || height <= 0 || width > 2048 || height > 1536 || data.size() < qsizetype(width) * height) return {};
  QImage image(width, height, QImage::Format_RGB888);
  const auto* src = reinterpret_cast<const quint8*>(data.constData());
  auto at = [&](int x, int y) -> quint8 { return src[std::clamp(y, 0, height - 1) * width + std::clamp(x, 0, width - 1)]; };
  for (int y = 0; y < height; ++y) {
    auto* out = image.scanLine(y);
    for (int x = 0; x < width; ++x) {
      const bool evenX = (x & 1) == 0, evenY = (y & 1) == 0;
      quint8 r, g, b;
      if (evenY && !evenX) { r = at(x,y); g = at(x-1,y); b = at(x-1,y+1); }
      else if (!evenY && evenX) { b = at(x,y); g = at(x+1,y); r = at(x+1,y-1); }
      else if (evenY) { g = at(x,y); r = at(x+1,y); b = at(x,y+1); }
      else { g = at(x,y); r = at(x,y-1); b = at(x-1,y); }
      out[x*3] = r; out[x*3+1] = g; out[x*3+2] = b;
    }
  }
  return image;
}

QImage decode(const QByteArray& payload, int width, int height, quint32 format) {
  if (format == 1005) return QImage::fromData(payload, "JPEG");
  if (format == 0x41524742u) {
    if (width <= 0 || height <= 0 || width > 2048 || height > 1536 || payload.size() < qsizetype(width)*height*4) return {};
    return QImage(reinterpret_cast<const uchar*>(payload.constData()), width, height, width*4, QImage::Format_ARGB32).copy();
  }
  if (format == 4) return bayerGrbg(payload, width, height);
  if (format == 5) {
    QImage infrared = grayPacked(payload, width, height, 10, false);
    return height == 488 && !infrared.isNull() ? infrared.copy(0, 4, width, 480) : infrared;
  }
  if (format == 6) return grayPacked(payload, width, height, 11, false);
  if (format == 1003 || format == 1004) return grayPacked(payload, width, height, 16, format == 1003);
  return {};
}

QByteArray metricDepth360(const QByteArray& payload, int width, int height, const QByteArray& reply) {
  const qsizetype pixels = qsizetype(width) * height;
  if (width <= 0 || height <= 0 || pixels > 2048 * 1536 ||
      payload.size() * 8 < pixels * 11 || reply.size() != 68 || !u32(reply.constData() + 32)) return {};
  const double shift = f64(reply.constData() + 36);
  const double emitter = f64(reply.constData() + 44);
  const double distance = f64(reply.constData() + 52);
  const double pixelSize = f64(reply.constData() + 60);
  if (!std::isfinite(shift) || !std::isfinite(emitter) || !std::isfinite(distance) ||
      !std::isfinite(pixelSize) || shift <= 0 || emitter <= 0 || distance <= 0 || pixelSize <= 0) return {};
  quint16 lookup[2048]{};
  for (int raw = 0; raw < 2047; ++raw) {
    const double metric = (((raw - 4.0 * shift) / 4.0) - 0.375) * pixelSize;
    const double denominator = emitter - metric;
    if (std::abs(denominator) < 1e-9) continue;
    const double mm = 10.0 * (metric * distance / denominator + distance);
    if (std::isfinite(mm) && mm >= 1 && mm <= 10000) lookup[raw] = quint16(std::lround(mm));
  }
  QByteArray result(pixels * 2, 0);
  const auto* src = reinterpret_cast<const quint8*>(payload.constData());
  for (qsizetype i = 0; i < pixels; ++i) {
    const qsizetype bit = i * 11;
    quint16 raw = 0;
    for (int b = 0; b < 11; ++b)
      raw = quint16((raw << 1) | ((src[(bit + b) / 8] >> (7 - (bit + b) % 8)) & 1u));
    const quint16 mm = lookup[raw];
    std::memcpy(result.data() + i * 2, &mm, sizeof(mm));
  }
  return result;
}

class StreamWorker : public QThread {
  Q_OBJECT
public:
  StreamWorker(Device device, quint32 mask, int overlay, QObject* parent = nullptr)
      : QThread(parent), device_(std::move(device)), mask_(mask), overlay_(overlay) {}
  ~StreamWorker() override {
    auto& leases=activeStreams();
    if(leases.value(device_.id)==this)leases.remove(device_.id);
  }
  void startLatest() {
    auto& leases=activeStreams();
    if(StreamWorker* previous=leases.value(device_.id);previous&&previous!=this) {
      previous->requestInterruption();previous->wait();
      emit previous->message("Sensor handed to the latest window");
    }
    leases[device_.id]=this;
    start();
  }
private:
  static QHash<QString,StreamWorker*>& activeStreams() {
    static QHash<QString,StreamWorker*> streams;
    return streams;
  }
signals:
  void frameReady(QImage image);
  void rgbReady(QImage image,quint64 tickMs);
  void depthStamp(quint64 tickMs);
  void depthReady(QByteArray depth, int width, int height, double fx, double fy,
                  double cx, double cy, QByteArray extension);
  void poseReady(QByteArray poseJson);
  void profileReady(double bodyMs,double handMs,double faceMs,double liftMs,int frames);
  void providerReady(QString body,QString hands,QString face);
  void message(QString text);
protected:
  void run() override {
#ifdef REMOLD_HAS_ONNX
    std::unique_ptr<OnnxBodyTracker> body;
    std::unique_ptr<OnnxHandTracker> hands;
    std::unique_ptr<OnnxFaceTracker> face;
    if (overlay_ == 1 || overlay_ == 5 || overlay_ == 6 || overlay_ == 7) {
#ifdef Q_OS_MACOS
      body = std::make_unique<OnnxBodyTracker>(
          QCoreApplication::applicationDirPath() + "/../Resources/models/movenet.onnx");
#else
      body = std::make_unique<OnnxBodyTracker>(
          QCoreApplication::applicationDirPath() + "/models/movenet.onnx");
#endif
      if (!body->ready()) emit message("MoveNet unavailable: " + body->error());
      if (overlay_ == 5 || overlay_ == 7) {
#ifdef Q_OS_MACOS
        const QString handPath = QCoreApplication::applicationDirPath() + "/../Resources/models/rtmpose-hand.onnx";
#else
        const QString handPath = QCoreApplication::applicationDirPath() + "/models/rtmpose-hand.onnx";
#endif
        hands = std::make_unique<OnnxHandTracker>(handPath);
        if (!hands->ready()) emit message("RTMPose Hand unavailable: " + hands->error());
      }
      if (overlay_ == 5 || overlay_ == 6) {
#ifdef Q_OS_MACOS
        const QString facePath=QCoreApplication::applicationDirPath()+"/../Resources/models/face-landmarks.onnx";
#else
        const QString facePath=QCoreApplication::applicationDirPath()+"/models/face-landmarks.onnx";
#endif
        face=std::make_unique<OnnxFaceTracker>(facePath);
        if(!face->ready())emit message("Face landmarks unavailable: "+face->error());
      }
    }
    std::unique_ptr<SynSkeletonTracker> skeleton;
    DepthCalibrationProfile skeletonCalibration(device_.id);
    bool skeletonCalibrationChecked = false;
    if (overlay_ == 4) {
#ifdef Q_OS_MACOS
      const QString modelPath = QCoreApplication::applicationDirPath() + "/../Resources/models/SynSkeleton.onnx";
#else
      const QString modelPath = QCoreApplication::applicationDirPath() + "/models/SynSkeleton.onnx";
#endif
      skeleton = std::make_unique<SynSkeletonTracker>(modelPath);
    }
#endif
#ifdef REMOLD_HAS_ONNX
    if(body)emit providerReady(body->provider(),hands?hands->provider():"Off",
        face?face->provider():"Off");
#endif
    QLocalSocket socket;
    QByteArray request;
    if (device_.generation == QStringLiteral("xbox-one")) {
      KinectOneRemoldProtocol::RequestV2 value{};
      value.request.streamMask = mask_;
      const QByteArray id = device_.id.toUtf8();
      const bool requestExtension = id.size() <= 62;
      std::memcpy(value.deviceId, id.constData(), std::min(size_t(id.size()), sizeof(value.deviceId)-1));
      if (requestExtension)
        value.deviceId[63] = static_cast<char>(KinectOneRemoldProtocol::kReplyExtensionFullRequest);
      request = QByteArray(reinterpret_cast<const char*>(&value), sizeof(value));
    } else {
      remold::scanner::Request value{}; value.streamMask = mask_;
      request = QByteArray(reinterpret_cast<const char*>(&value), sizeof(value));
    }
    QByteArray reply;
    for (int attempt = 0; attempt < 8 && !isInterruptionRequested(); ++attempt) {
      socket.connectToServer(device_.camera);
      if (!socket.waitForConnected(2000)) { emit message(socket.errorString()); return; }
      socket.write(request);
      if (!socket.waitForBytesWritten(2000)) { emit message("Stream request failed"); return; }
      reply = readExact(socket, 68, this);
      if (reply.size() != 68 || device_.generation != QStringLiteral("xbox-one")) break;
      const qint32 result=qint32(u32(reply.constData()+8));
      if (result != KinectOneRemoldProtocol::ResultDeviceBusy &&
          result != KinectOneRemoldProtocol::ResultStreamingUnavailable) break;
      socket.abort();
      emit message(result==KinectOneRemoldProtocol::ResultDeviceBusy
          ? "Sensor is busy; retrying…" : "Sensor stream startup failed; retrying…");
      QThread::msleep(result==KinectOneRemoldProtocol::ResultDeviceBusy?250:1000);
    }
    if (reply.size() != 68 || u32(reply.constData()) != 0x43534D52u ||
        u32(reply.constData()+4) != 1 || u32(reply.constData()+8) != 0 ||
        u32(reply.constData()+12) != mask_) {
      const qint32 result=reply.size()==68?qint32(u32(reply.constData()+8)):999;
      const QString reason=result==KinectOneRemoldProtocol::ResultDeviceBusy?"sensor is busy":
          result==KinectOneRemoldProtocol::ResultDeviceNotFound?"sensor disconnected":
          result==KinectOneRemoldProtocol::ResultStreamingUnavailable?
              "USB stream startup failed; inspect native-service terminal":
          result==KinectOneRemoldProtocol::ResultUnsupported?"unsupported stream request":
          result==999?"incomplete reply":"invalid reply";
      emit message(QString("Cannot start tracking: %1 (code %2)").arg(reason).arg(result));return;
    }
    QByteArray extension;
    const bool nativeDepth = device_.generation == QStringLiteral("xbox-one") &&
        u32(reply.constData() + 32) >= KinectOneRemoldProtocol::kDepthCalibrationNative;
    const std::uint32_t calibrationCode=u32(reply.constData()+32);
    if (nativeDepth && (calibrationCode == KinectOneRemoldProtocol::kDepthCalibrationNativeExtended ||
                        calibrationCode == KinectOneRemoldProtocol::kDepthCalibrationNativeFull)) {
      const int bytes=calibrationCode == KinectOneRemoldProtocol::kDepthCalibrationNativeFull
          ? sizeof(KinectOneRemoldProtocol::ReplyExtensionV2)
          : sizeof(KinectOneRemoldProtocol::ReplyExtension);
      extension = readExact(socket, bytes, this);
      if (extension.size() != bytes ||
          u32(extension.constData()) != KinectOneRemoldProtocol::kReplyExtensionMagic) {
        emit message("Invalid factory calibration reply"); return;
      }
    }
    emit message("Streaming " + device_.label);
 #ifdef REMOLD_HAS_ONNX
    if (overlay_ == 4 && skeleton && !skeleton->ready())
      emit message("SynSkeleton unavailable: " + skeleton->error());
    if ((overlay_ == 5 || overlay_ == 7) && hands && !hands->ready())
      emit message("RTMPose Hand unavailable: " + hands->error());
 #endif
    QByteArray latestDepth;
    DepthIntrinsics latestCamera;
    quint64 latestDepthTick=0;
    quint64 lastPoseTick=0;
    DepthCalibrationProfile poseCalibration(device_.id);
    bool poseCalibrationChecked=false;
    float personDepth=0;
    int personDepthMisses=0;
    const bool profileMocap=!qEnvironmentVariableIsEmpty("REMOLD_PROFILE_MOCAP");
    double bodyMs=0,handMs=0,faceMs=0,liftMs=0;
    int profileFrames=0;
    while (!isInterruptionRequested()) {
      const QByteArray header = readExact(socket, 76, this);
      if (header.size() != 76) break;
      const quint32 magic = u32(header.constData());
      const int width = int(u32(header.constData()+12)), height = int(u32(header.constData()+16));
      const quint32 format = u32(header.constData()+20), bytes = u32(header.constData()+24);
      const quint64 frameNumber=u64(header.constData()+32);
      const quint64 frameTick=u64(header.constData()+40);
      if (magic != 0x46534D52u || bytes > 16u*1024u*1024u) break;
      const QByteArray payload = readExact(socket, bytes, this);
      if (payload.size() != bytes) break;
      if (nativeDepth && format == KinectOneRemoldProtocol::DepthMm16 &&
          payload.size() == qsizetype(width) * height * 2) {
        latestDepth=payload;
        latestCamera={width,height,f64(reply.constData()+36),f64(reply.constData()+44),
                      f64(reply.constData()+52),f64(reply.constData()+60),extension};
        latestDepthTick=frameTick;
        emit depthStamp(frameTick);
        emit depthReady(payload, width, height, f64(reply.constData()+36),
                        f64(reply.constData()+44), f64(reply.constData()+52),
                        f64(reply.constData()+60), extension);
      }
      if (device_.generation == QStringLiteral("xbox-360") && format == 6) {
        const QByteArray metric = metricDepth360(payload, width, height, reply);
        if (!metric.isEmpty()) {
          latestDepth=metric;latestCamera={width,height,594.2143421192325,
              591.0405369687078,339.30780975300314,242.73913761751615,{}};
          latestDepthTick=frameTick;
          emit depthStamp(frameTick);
          emit depthReady(metric, width, height, 594.2143421192325,
                          591.0405369687078, 339.30780975300314,
                          242.73913761751615, {});
        }
      }
      // Retain metric depth for export while the landmark preview stays RGB.
      if (overlay_ > 0 && overlay_ != 4 && device_.generation == QStringLiteral("xbox-one") &&
          format != KinectOneRemoldProtocol::Jpeg) continue;
      QImage image = decode(payload, width, height, format);
      if(!image.isNull()&&(format==KinectOneRemoldProtocol::Jpeg||
                            format==4||format==0x41524742u))emit rgbReady(image,frameTick);
      if (!image.isNull() && overlay_ > 0) {
#ifdef REMOLD_HAS_ONNX
        if (overlay_ == 1 && body && body->ready()) image = body->annotate(image);
        if ((overlay_ == 5 || overlay_ == 6 || overlay_ == 7) && body && body->ready()) {
          QElapsedTimer phase; if(profileMocap)phase.start();
          const QImage original=image;
          const QImage bodyImage=body->annotate(image);
          if(profileMocap){bodyMs+=phase.nsecsElapsed()*1e-6;phase.restart();}
          if(overlay_!=6)image=bodyImage;
          if (hands && hands->ready()) image = hands->annotate(image, body->keypoints());
          if(profileMocap){handMs+=phase.nsecsElapsed()*1e-6;phase.restart();}
          QJsonObject facial=face&&face->ready()?
              face->track(original,body->keypoints()):QJsonObject{};
          if(profileMocap){faceMs+=phase.nsecsElapsed()*1e-6;phase.restart();}
          QJsonArray imageBody;
          if(overlay_!=6)for(const BodyKeypoint& keypoint:body->keypoints())
            imageBody.append(QJsonArray{keypoint.x,keypoint.y,keypoint.confidence});
          if(facial.value("tracked").toBool()) {
            QPainter painter(&image);painter.setPen(QPen(QColor(246,105,192),2));
            painter.setBrush(QColor(246,105,192));
            const QJsonArray points=facial.value("landmarks").toArray();
            for(int i=0;i<points.size();i+=4) {
              const QJsonArray point=points[i].toArray();
              if(point.size()==3)painter.drawEllipse(QPointF(point[0].toDouble()*image.width(),
                  point[1].toDouble()*image.height()),2,2);
            }
          }
          if(overlay_==6) {
            QJsonObject root{{"v",1},{"device",device_.id},
                {"t",double(frameTick)*.001},{"frame",double(frameNumber)},
                {"space","camera"},{"mirror",false},{"tracked",false},
                {"observed",0},{"bodies",QJsonArray{}}};
            if(!facial.isEmpty())root["face"]=facial;
            emit frameReady(image);
            emit poseReady(QJsonDocument(root).toJson(QJsonDocument::Compact));
            if(profileMocap&&++profileFrames%60==0)
              emit profileReady(bodyMs/profileFrames,handMs/profileFrames,
                  faceMs/profileFrames,0,profileFrames);
          } else {
          const quint64 skew=frameTick>latestDepthTick?frameTick-latestDepthTick:
              latestDepthTick-frameTick;
          if(!latestDepth.isEmpty()&&skew<=100) {
            if(!poseCalibrationChecked) {
              poseCalibrationChecked=true;
              poseCalibration.load(calibrationPath(device_.id),device_.id,
                                   latestCamera.width,latestCamera.height);
            }
            if(profileMocap)phase.restart();
            const auto pose=RgbdRegistration::lift(latestDepth,latestCamera,&poseCalibration,
                image.width(),image.height(),device_.generation==QStringLiteral("xbox-one"),
                body->keypoints(),hands?hands->keypoints():
                    std::array<std::array<BodyKeypoint,21>,2>{},personDepth);
            if(profileMocap){liftMs+=phase.nsecsElapsed()*1e-6;
              if(++profileFrames%60==0) {
                qInfo("Mocap stage ms/frame (%d): body %.2f hands %.2f face %.2f RGBD lift %.2f",
                    profileFrames,bodyMs/profileFrames,handMs/profileFrames,
                    faceMs/profileFrames,liftMs/profileFrames);
                emit profileReady(bodyMs/profileFrames,handMs/profileFrames,
                    faceMs/profileFrames,liftMs/profileFrames,profileFrames);
              }
            }
            std::vector<float> torsoDepths;
            for(int index:{4,5,10,11})
              if(pose[index].valid&&pose[index].confidence>=.2f)
                torsoDepths.push_back(pose[index].point.z());
            if(torsoDepths.size()>=3) {
              std::sort(torsoDepths.begin(),torsoDepths.end());
              const float measured=torsoDepths[torsoDepths.size()/2];
              if(personDepth==0||std::abs(measured-personDepth)<.35f) {
                personDepth=personDepth==0?measured:personDepth*.94f+measured*.06f;
                personDepthMisses=0;
              } else if(++personDepthMisses>30)personDepth=0;
            } else if(++personDepthMisses>30)personDepth=0;
            if(!facial.isEmpty()&&pose[1].valid) {
              QJsonObject head=facial.value("head").toObject();
              head["pCamera"]=QJsonArray{pose[1].point.x(),pose[1].point.y(),pose[1].point.z()};
              facial["head"]=head;
            }
            int torso=0;
            for(int index:{4,5,10,11})
              if(pose[index].valid&&pose[index].confidence>=.2f) ++torso;
            const bool tracked=torso>=3;
            static const char* names[]{"head","nose","earL","earR","shoulderL","shoulderR",
                "elbowL","elbowR","wristL","wristR","hipL","hipR","kneeL","kneeR",
                "ankleL","ankleR","handL_0","handL_1","handL_2","handL_3","handL_4",
                "handL_5","handL_6","handL_7","handL_8","handL_9","handL_10","handL_11",
                "handL_12","handL_13","handL_14","handL_15","handL_16","handL_17",
                "handL_18","handL_19","handL_20","handR_0","handR_1","handR_2",
                "handR_3","handR_4","handR_5","handR_6","handR_7","handR_8","handR_9",
                "handR_10","handR_11","handR_12","handR_13","handR_14","handR_15",
                "handR_16","handR_17","handR_18","handR_19","handR_20"};
            QJsonObject joints;int count=0;
            const int bodyMap[16]{0,0,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
            for(int i=0;i<RgbdRegistration::jointCount;++i) if(tracked&&pose[i].valid) {
              const auto& p=pose[i].point;
              const BodyKeypoint uv=i<16?body->keypoints()[bodyMap[i]]:
                  hands->keypoints()[(i-16)/21][(i-16)%21];
              joints[names[i]]=QJsonObject{{"p",QJsonArray{p.x(),p.y(),p.z()}},
                  {"uv",QJsonArray{uv.x,uv.y}},
                  {"c",pose[i].confidence},{"s",2}};
              ++count;
            }
            QJsonArray bodies;
            if(tracked) bodies.append(QJsonObject{{"id",1},{"tracked",true},
                {"confidence",(pose[4].confidence+pose[5].confidence+
                               pose[10].confidence+pose[11].confidence)*.25f},
                {"joints",joints}});
            QJsonObject root{{"v",1},{"device",device_.id},{"t",double(frameTick)*.001},
                {"frame",double(frameNumber)},
                {"space","camera"},{"mirror",false},
                {"registration",latestCamera.extension.size()>=144&&
                    (quint8(latestCamera.extension[12])&4u)?"factory-polynomial":"nominal-color-projection"},
                {"tracked",tracked},{"observed",count},{"bodies",bodies}};
            root["imageBody"]=imageBody;
            if(!facial.isEmpty())root["face"]=facial;
            emit frameReady(image);
            emit poseReady(QJsonDocument(root).toJson(QJsonDocument::Compact));
            lastPoseTick=frameTick;
          } else {
            // RGB can arrive between depth frames. Preserve the last valid
            // solve through a short gap instead of flashing "lost" every time.
            if(lastPoseTick&&frameTick>=lastPoseTick&&frameTick-lastPoseTick<500)
              continue;
            QJsonObject root{{"v",1},{"device",device_.id},
                {"t",double(frameTick)*.001},{"frame",double(frameNumber)},
                {"space","camera"},{"mirror",false},{"tracked",false},
                {"observed",0},{"bodies",QJsonArray{}},{"imageBody",imageBody}};
            if(!facial.isEmpty())root["face"]=facial;
            emit frameReady(image);
            emit poseReady(QJsonDocument(root).toJson(QJsonDocument::Compact));
          }
          }
        }
        if (overlay_ == 4 && skeleton && skeleton->ready() &&
            (format == KinectOneRemoldProtocol::DepthMm16 || format == 6)) {
          const QByteArray metric = format == 6 ? metricDepth360(payload, width, height, reply) : payload;
          DepthIntrinsics camera{width, height,
              format == 6 ? 594.2143421192325 : f64(reply.constData()+36),
              format == 6 ? 591.0405369687078 : f64(reply.constData()+44),
              format == 6 ? 339.30780975300314 : f64(reply.constData()+52),
              format == 6 ? 242.73913761751615 : f64(reply.constData()+60), extension};
          if (!skeletonCalibrationChecked) {
            skeletonCalibrationChecked = true;
            skeletonCalibration.load(calibrationPath(device_.id), device_.id, width, height);
          }
          std::array<SynSkeletonJoint, SynSkeletonTracker::jointCount> joints;
          if (skeleton->infer(metric, camera, &skeletonCalibration, joints))
            image = SynSkeletonTracker::annotate(image, joints, camera);
        }
#endif
#ifdef Q_OS_MACOS
        if (overlay_ == 2 || overlay_ == 3)
          image = annotateWithVision(image, static_cast<VisionOverlay>(overlay_ - 1));
#endif
      }
      if (!image.isNull() && (overlay_<5 || overlay_>7)) emit frameReady(image);
    }
    emit message("Stream stopped");
  }
private:
  Device device_;
  quint32 mask_;
  int overlay_;
};

class AudioWorker : public QThread {
  Q_OBJECT
 public:
  explicit AudioWorker(Device device, QObject* parent = nullptr) : QThread(parent), device_(std::move(device)) {}
 signals:
  void audioReady(QByteArray samples, QByteArray beam, QVector<int> levels,
                  float azimuthDeg, float confidence, float rms);
  void message(QString text);
 protected:
  void run() override {
    QLocalSocket socket;
    socket.connectToServer(device_.audio);
    if (!socket.waitForConnected(2000)) { emit message(socket.errorString()); return; }
    remold::audio::Request request{};
    if (socket.write(reinterpret_cast<const char*>(&request), sizeof(request)) != sizeof(request) ||
        !socket.waitForBytesWritten(2000)) { emit message("Audio request failed"); return; }
    const QByteArray replyBytes = readExact(socket, sizeof(remold::audio::Reply), this);
    if (replyBytes.size() != sizeof(remold::audio::Reply)) { emit message("Audio reply timed out"); return; }
    remold::audio::Reply reply{};
    std::memcpy(&reply, replyBytes.constData(), sizeof(reply));
    if (reply.magic != remold::audio::kMagic || reply.version != remold::audio::kVersion ||
        reply.result != 0 || reply.sampleRate != 16000 || reply.channels != 4 ||
        reply.sampleFormat != remold::audio::SampleFormat::PcmS32Le) {
      emit message("Audio transport rejected four-channel PCM"); return;
    }
    emit message("Capturing four-channel audio from " + device_.label);
    SpatialAudio spatial;
    while (!isInterruptionRequested()) {
      const QByteArray headerBytes = readExact(socket, sizeof(remold::audio::FrameHeader), this);
      if (headerBytes.size() != sizeof(remold::audio::FrameHeader)) break;
      remold::audio::FrameHeader header{};
      std::memcpy(&header, headerBytes.constData(), sizeof(header));
      if (header.magic != remold::audio::kFrameMagic || header.version != remold::audio::kVersion ||
          header.sampleRate != 16000 || header.channels != 4 ||
          header.sampleFormat != remold::audio::SampleFormat::PcmS32Le ||
          header.samplesPerChannel != remold::audio::kSamples ||
          header.payloadBytes != header.samplesPerChannel * 16) {
        emit message("Invalid audio frame"); break;
      }
      const QByteArray samples = readExact(socket, header.payloadBytes, this);
      if (samples.size() != header.payloadBytes) break;
      double squared[4]{};
      for (quint32 i = 0; i < header.samplesPerChannel; ++i) for (int channel = 0; channel < 4; ++channel) {
        qint32 sample;
        std::memcpy(&sample, samples.constData() + (i * 4 + channel) * 4, sizeof(sample));
        const double normalized = double(sample) / 2147483648.0;
        squared[channel] += normalized * normalized;
      }
      QVector<int> levels(4);
      for (int channel = 0; channel < 4; ++channel)
        levels[channel] = (header.channelMask & (1u << channel)) ?
            int(std::clamp(std::sqrt(squared[channel] / header.samplesPerChannel) * 250.0, 0.0, 100.0)) : 0;
      const SpatialAudioResult scan = spatial.process(samples, header.channelMask);
      emit audioReady(samples, scan.beamPcm16, levels, scan.azimuthDeg, scan.confidence, scan.rms);
    }
    emit message("Audio stopped");
  }
 private:
  Device device_;
};

class MicrophoneWindow : public QMainWindow {
 public:
  MicrophoneWindow() {
    setWindowTitle("Kinect Remold — Microphones");
    resize(620, 420);
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    devices_ = new QComboBox;
    layout->addWidget(devices_);
    auto* controls = new QHBoxLayout;
    auto* refreshButton = new QPushButton("Refresh");
    auto* start = new QPushButton("Start capture");
    auto* stop = new QPushButton("Stop");
    auto* record = new QPushButton("Record WAV");
    auto* beamRecord = new QPushButton("Record beam WAV");
    auto* finish = new QPushButton("Finish recording");
    controls->addWidget(refreshButton); controls->addWidget(start);
    controls->addWidget(stop); controls->addWidget(record);
    controls->addWidget(beamRecord); controls->addWidget(finish);
    layout->addLayout(controls);
    for (int channel = 0; channel < 4; ++channel) {
      auto* row = new QHBoxLayout;
      row->addWidget(new QLabel(QString("Microphone %1").arg(channel + 1)));
      meters_[channel] = new QProgressBar;
      meters_[channel]->setRange(0, 100);
      row->addWidget(meters_[channel], 1);
      layout->addLayout(row);
    }
    auto* gain = new QHBoxLayout;
    gain->addWidget(new QLabel("Input volume"));
    volume_ = new QSlider(Qt::Horizontal);
    volume_->setRange(0, 100); volume_->setValue(100);
    gain->addWidget(volume_, 1);
    mute_ = new QCheckBox("Mute");
    gain->addWidget(mute_);
    layout->addLayout(gain);
    direction_ = new QLabel("Sound direction: —");
    layout->addWidget(direction_);
    status_ = new QLabel;
    layout->addWidget(status_);
    setCentralWidget(central);
    connect(refreshButton, &QPushButton::clicked, this, [this] { refresh(); });
    connect(start, &QPushButton::clicked, this, [this] { startCapture(); });
    connect(stop, &QPushButton::clicked, this, [this] { stopCapture(); });
    connect(record, &QPushButton::clicked, this, [this] { startRecording(false); });
    connect(beamRecord, &QPushButton::clicked, this, [this] { startRecording(true); });
    connect(finish, &QPushButton::clicked, this, [this] { finishRecording(); });
    connect(volume_, &QSlider::sliderReleased, this, [this] {
      if (const Device* device = selected())
        status_->setText(audioControlAt(*device, remold::audio_control::Command::SetVolume, volume_->value()));
    });
    connect(mute_, &QCheckBox::toggled, this, [this](bool muted) {
      if (const Device* device = selected())
        status_->setText(audioControlAt(*device, remold::audio_control::Command::SetMute, muted ? 1 : 0));
    });
    connect(devices_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
      const Device* device = selected();
      const bool controllable = device && !device->audioControl.isEmpty();
      volume_->setEnabled(controllable); mute_->setEnabled(controllable);
    });
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { if (!worker_) refresh(); });
    timer->start(2000);
    refresh();
  }
  ~MicrophoneWindow() override { stopCapture(); }
  int checkLiveAudio() {
    if (found_.empty()) return 2;
    startCapture();
    QElapsedTimer timeout; timeout.start();
    while (timeout.elapsed() < 5000 && frames_ == 0) {
      QApplication::processEvents();
      QThread::msleep(20);
    }
    return frames_ > 0 ? 0 : 3;
  }
 private:
  const Device* selected() const {
    const int index = devices_->currentIndex();
    return index >= 0 && index < int(found_.size()) ? &found_[size_t(index)] : nullptr;
  }
  void refresh() {
    if (worker_) return;
    const QString id = devices_->currentData().toString();
    found_ = discoverAt(sdkPath(false));
    auto one = discoverAt(sdkPath(true)); found_.insert(found_.end(), one.begin(), one.end());
    found_.erase(std::remove_if(found_.begin(), found_.end(), [](const Device& device) {
      return device.audio.isEmpty() || device.state != QStringLiteral("Ready");
    }), found_.end());
    devices_->clear();
    for (const Device& device : found_) devices_->addItem(device.label, device.id);
    const int index = devices_->findData(id);
    if (index >= 0) devices_->setCurrentIndex(index);
    status_->setText(found_.empty() ? "No four-channel audio endpoint available" :
                                   QString("%1 audio device(s) found").arg(found_.size()));
  }
  void startCapture() {
    stopCapture();
    const Device* device = selected();
    if (!device) { status_->setText("Select an audio device"); return; }
    worker_ = new AudioWorker(*device, this);
    connect(worker_, &AudioWorker::message, status_, &QLabel::setText);
    connect(worker_, &AudioWorker::audioReady, this,
            [this](const QByteArray& samples, const QByteArray& beam, const QVector<int>& levels,
                   float azimuth, float confidence, float) {
      ++frames_;
      for (int channel = 0; channel < 4 && channel < levels.size(); ++channel)
        meters_[channel]->setValue(levels[channel]);
      direction_->setText(QString("Sound direction: %1° · confidence %2%")
          .arg(azimuth, 0, 'f', 1).arg(int(confidence * 100)));
      const QByteArray& output = beamMode_ ? beam : samples;
      if (recording_ && recording_->write(output) != output.size()) {
        status_->setText(recording_->errorString()); finishRecording();
      }
    });
    worker_->start();
  }
  void stopCapture() {
    finishRecording();
    if (!worker_) return;
    worker_->requestInterruption();
    worker_->wait();
    delete worker_; worker_ = nullptr;
    for (auto* meter : meters_) meter->setValue(0);
  }
  void startRecording(bool beam) {
    if (!worker_ || !worker_->isRunning()) { status_->setText("Start audio capture first"); return; }
    finishRecording();
    const QString suggested = QStandardPaths::writableLocation(QStandardPaths::MusicLocation) +
        (beam ? "/Kinect-beam-" : "/Kinect-audio-") +
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") + ".wav";
    const QString path = QFileDialog::getSaveFileName(this, beam ? "Record beamformed audio" :
                                                     "Record four-channel audio", suggested,
                                                     "WAV audio (*.wav)");
    if (path.isEmpty()) return;
    recording_ = std::make_unique<QSaveFile>(path);
    beamMode_ = beam;
    if (!recording_->open(QIODevice::WriteOnly)) {
      status_->setText(recording_->errorString()); recording_.reset(); return;
    }
    QByteArray header(44, 0);
    auto put16 = [&](int offset, quint16 value) { std::memcpy(header.data() + offset, &value, 2); };
    auto put32 = [&](int offset, quint32 value) { std::memcpy(header.data() + offset, &value, 4); };
    std::memcpy(header.data(), "RIFF", 4); std::memcpy(header.data() + 8, "WAVEfmt ", 8);
    put32(16, 16); put16(20, 1); put16(22, beam ? 1 : 4); put32(24, 16000);
    put32(28, beam ? 32000 : 256000); put16(32, beam ? 2 : 16);
    put16(34, beam ? 16 : 32);
    std::memcpy(header.data() + 36, "data", 4);
    if (recording_->write(header) != header.size()) {
      status_->setText(recording_->errorString()); recording_.reset(); return;
    }
    status_->setText(beam ? "Recording beamformed WAV" : "Recording four-channel WAV");
  }
  void finishRecording() {
    if (!recording_) return;
    const qint64 bytes = recording_->size() - 44;
    if (bytes < 0 || bytes > 0xffffffffu - 36u || !recording_->seek(4)) {
      status_->setText("WAV recording exceeded the format limit");
      recording_.reset(); return;
    }
    const quint32 riff = quint32(bytes + 36), data = quint32(bytes);
    if (recording_->write(reinterpret_cast<const char*>(&riff), 4) != 4 ||
        !recording_->seek(40) ||
        recording_->write(reinterpret_cast<const char*>(&data), 4) != 4 ||
        !recording_->commit()) status_->setText("Could not finalize WAV recording");
    else status_->setText(QString("Saved %1 bytes of four-channel audio").arg(bytes));
    recording_.reset();
  }
  std::vector<Device> found_;
  AudioWorker* worker_ = nullptr;
  std::unique_ptr<QSaveFile> recording_;
  QComboBox* devices_ = nullptr;
  QProgressBar* meters_[4]{};
  QSlider* volume_ = nullptr;
  QCheckBox* mute_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* direction_ = nullptr;
  bool beamMode_ = false;
  quint64 frames_ = 0;
};

class AcousticView : public QWidget {
 public:
  explicit AcousticView(QWidget* parent = nullptr) : QWidget(parent) {
    setMinimumSize(600, 360);
    occupancy_.fill(0);
  }
  void updateScan(float azimuth, float confidence, float rms) {
    for (float& value : occupancy_) value *= 0.94f;
    if (rms >= 0.0018f && confidence >= 0.015f) {
      direction_ = std::clamp(azimuth, -90.0f, 90.0f);
      confidence_ = confidence;
      const int center = std::clamp(int(std::lround(direction_)) + 90, 0, 180);
      for (int delta = -8; delta <= 8; ++delta) {
        const int bin = center + delta;
        if (bin < 0 || bin >= int(occupancy_.size())) continue;
        const float contribution = confidence * std::exp(-0.5f * delta * delta / 12.0f);
        occupancy_[size_t(bin)] = std::clamp(occupancy_[size_t(bin)] + contribution * 0.16f, 0.0f, 1.0f);
      }
    }
    update();
  }
 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor(19, 26, 34));
    const QPointF origin(width() / 2.0, height() - 34.0);
    const float radius = std::min(width() * 0.43f, height() - 64.0f);
    painter.setPen(QPen(QColor(80, 95, 110), 1));
    for (float fraction : {0.25f, 0.5f, 0.75f, 1.0f}) {
      const QRectF circle(origin.x() - radius * fraction, origin.y() - radius * fraction,
                          radius * fraction * 2, radius * fraction * 2);
      painter.drawArc(circle, 0, 180 * 16);
    }
    for (int degree = -90; degree <= 90; degree += 30) {
      const float radians = degree * 3.14159265358979323846f / 180;
      const QPointF end(origin.x() + std::sin(radians) * radius,
                        origin.y() - std::cos(radians) * radius);
      painter.drawLine(origin, end);
      painter.drawText(end + QPointF(-12, -6), QString::number(degree) + "°");
    }
    for (int bin = 0; bin < 181; ++bin) {
      const float radians = (bin - 90) * 3.14159265358979323846f / 180;
      const float occupancy = occupancy_[size_t(bin)];
      if (occupancy < 0.015f) continue;
      painter.setPen(QPen(QColor(30 + int(occupancy * 220), 155 + int(occupancy * 90), 190), 2));
      painter.drawLine(QPointF(origin.x() + std::sin(radians) * radius * 0.42f,
                               origin.y() - std::cos(radians) * radius * 0.42f),
                       QPointF(origin.x() + std::sin(radians) * radius * (0.42f + occupancy * 0.55f),
                               origin.y() - std::cos(radians) * radius * (0.42f + occupancy * 0.55f)));
    }
    const float radians = direction_ * 3.14159265358979323846f / 180;
    painter.setPen(QPen(QColor(48, 239, 177), 3));
    painter.drawLine(origin, QPointF(origin.x() + std::sin(radians) * radius * 0.9f,
                                     origin.y() - std::cos(radians) * radius * 0.9f));
    painter.setPen(Qt::white);
    painter.drawText(16, 24, QString("Azimuth %1° · confidence %2%")
        .arg(direction_, 0, 'f', 1).arg(int(confidence_ * 100)));
  }
 private:
  std::array<float, 181> occupancy_{};
  float direction_ = 0, confidence_ = 0;
};

class AcousticWindow : public QMainWindow {
 public:
  AcousticWindow() {
    setWindowTitle("Kinect Remold — Acoustic Scanner");
    resize(760, 500);
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    auto* controls = new QHBoxLayout;
    devices_ = new QComboBox;
    auto* refreshButton = new QPushButton("Refresh");
    auto* start = new QPushButton("Start scan");
    auto* stop = new QPushButton("Stop");
    controls->addWidget(devices_, 1); controls->addWidget(refreshButton);
    controls->addWidget(start); controls->addWidget(stop);
    layout->addLayout(controls);
    view_ = new AcousticView;
    layout->addWidget(view_, 1);
    status_ = new QLabel;
    layout->addWidget(status_);
    setCentralWidget(central);
    connect(refreshButton, &QPushButton::clicked, this, [this] { refresh(); });
    connect(start, &QPushButton::clicked, this, [this] { startScan(); });
    connect(stop, &QPushButton::clicked, this, [this] { stopScan(); });
    refresh();
  }
  ~AcousticWindow() override { stopScan(); }
  int checkLiveScan() {
    if (found_.empty()) return 2;
    startScan();
    QElapsedTimer timeout; timeout.start();
    while (timeout.elapsed() < 5000 && frames_ == 0) {
      QApplication::processEvents(); QThread::msleep(20);
    }
    return frames_ > 0 && !view_->grab().isNull() ? 0 : 3;
  }
 private:
  void refresh() {
    if (worker_) return;
    const QString selected = devices_->currentData().toString();
    found_ = discoverAt(sdkPath(false));
    auto one = discoverAt(sdkPath(true)); found_.insert(found_.end(), one.begin(), one.end());
    found_.erase(std::remove_if(found_.begin(), found_.end(), [](const Device& device) {
      return device.audio.isEmpty() || device.state != QStringLiteral("Ready");
    }), found_.end());
    devices_->clear();
    for (const Device& device : found_) devices_->addItem(device.label, device.id);
    const int index = devices_->findData(selected);
    if (index >= 0) devices_->setCurrentIndex(index);
    status_->setText(found_.empty() ? "No microphone array endpoint available" :
                                   QString("%1 microphone array(s) found").arg(found_.size()));
  }
  void startScan() {
    stopScan();
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size())) return;
    worker_ = new AudioWorker(found_[size_t(index)], this);
    connect(worker_, &AudioWorker::message, status_, &QLabel::setText);
    connect(worker_, &AudioWorker::audioReady, this,
            [this](const QByteArray&, const QByteArray&, const QVector<int>&,
                   float azimuth, float confidence, float rms) {
      ++frames_;
      view_->updateScan(azimuth, confidence, rms);
    });
    worker_->start();
  }
  void stopScan() {
    if (!worker_) return;
    worker_->requestInterruption(); worker_->wait();
    delete worker_; worker_ = nullptr;
  }
  std::vector<Device> found_;
  AudioWorker* worker_ = nullptr;
  QComboBox* devices_ = nullptr;
  AcousticView* view_ = nullptr;
  QLabel* status_ = nullptr;
  quint64 frames_ = 0;
};

QString calibrationPath(const QString& deviceId) {
  QString safe = deviceId;
  safe.replace(QRegularExpression("[^A-Za-z0-9_-]"), "_");
  return QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
      "/calibration/" + safe + ".json";
}

class CalibrationWindow : public QMainWindow {
 public:
  CalibrationWindow() {
    setWindowTitle("Kinect Remold — Sensor Calibration"); resize(860, 680);
    auto* central = new QWidget(this); auto* layout = new QVBoxLayout(central);
    auto* top = new QHBoxLayout;
    devices_ = new QComboBox;
    auto* refreshButton = new QPushButton("Refresh");
    auto* start = new QPushButton("Start depth"); auto* stop = new QPushButton("Stop");
    top->addWidget(devices_, 1); top->addWidget(refreshButton);
    top->addWidget(start); top->addWidget(stop); layout->addLayout(top);
    auto* actions = new QHBoxLayout;
    tape_ = new QDoubleSpinBox;
    tape_->setRange(0, 6); tape_->setDecimals(2); tape_->setSingleStep(0.05);
    tape_->setSuffix(" m"); tape_->setSpecialValueText("No tape reference");
    auto* capture = new QPushButton("Capture wall station");
    auto* validate = new QPushButton("Validate");
    auto* save = new QPushButton("Save calibration");
    auto* reset = new QPushButton("Reset");
    actions->addWidget(new QLabel("Tape distance")); actions->addWidget(tape_);
    actions->addWidget(capture); actions->addWidget(validate);
    actions->addWidget(save); actions->addWidget(reset); actions->addStretch();
    layout->addLayout(actions);
    progress_ = new QProgressBar; progress_->setRange(0, 30); layout->addWidget(progress_);
    preview_ = new QLabel("Start metric depth, then face a flat wall.");
    preview_->setAlignment(Qt::AlignCenter); preview_->setMinimumSize(512, 424);
    preview_->setStyleSheet("background:#151b22;color:#e5e9ef;");
    layout->addWidget(preview_, 1);
    status_ = new QLabel; layout->addWidget(status_);
    setCentralWidget(central);
    connect(refreshButton, &QPushButton::clicked, this, [this] { refresh(); });
    connect(start, &QPushButton::clicked, this, [this] { startDepth(); });
    connect(stop, &QPushButton::clicked, this, [this] { stopDepth(); });
    connect(capture, &QPushButton::clicked, this, [this] { beginCapture(Mode::Station); });
    connect(validate, &QPushButton::clicked, this, [this] { beginCapture(Mode::Validation); });
    connect(save, &QPushButton::clicked, this, [this] { saveProfile(); });
    connect(reset, &QPushButton::clicked, this, [this] { resetProfile(); });
    connect(devices_, QOverload<int>::of(&QComboBox::activated), this, [this](int) {
      stopDepth(); profile_ = DepthCalibrationProfile(devices_->currentData().toString());
      intrinsics_ = {}; profileInitialized_ = validated_ = false;
    });
    refresh();
  }
  ~CalibrationWindow() override { stopDepth(); }
 private:
  enum class Mode { None, Station, Validation };
  void refresh() {
    if (worker_) return;
    const QString selected = devices_->currentData().toString();
    found_ = discoverAt(sdkPath(false));
    auto one = discoverAt(sdkPath(true)); found_.insert(found_.end(), one.begin(), one.end());
    found_.erase(std::remove_if(found_.begin(), found_.end(), [](const Device& device) {
      return device.camera.isEmpty() || device.state != QStringLiteral("Ready");
    }), found_.end());
    devices_->clear();
    for (const Device& device : found_) devices_->addItem(device.label, device.id);
    const int index = devices_->findData(selected);
    if (index >= 0) devices_->setCurrentIndex(index);
    status_->setText(found_.empty() ? "No metric depth camera available" :
                                   QString("%1 depth camera(s) found").arg(found_.size()));
  }
  void startDepth() {
    stopDepth();
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size())) return;
    const Device device = found_[size_t(index)];
    if (!profileInitialized_) profile_ = DepthCalibrationProfile(device.id);
    worker_ = new StreamWorker(device, 4u, 0, this);
    connect(worker_, &StreamWorker::message, status_, &QLabel::setText);
    connect(worker_, &StreamWorker::frameReady, this, [this](const QImage& image) {
      preview_->setPixmap(QPixmap::fromImage(image).scaled(preview_->size(), Qt::KeepAspectRatio,
                                                          Qt::SmoothTransformation));
    });
    connect(worker_, &StreamWorker::depthReady, this,
            [this](const QByteArray& depth, int width, int height, double fx, double fy,
                   double cx, double cy, const QByteArray& extension) {
      if (!intrinsics_.valid() || intrinsics_.width != width || intrinsics_.height != height) {
        intrinsics_ = {width, height, fx, fy, cx, cy, extension};
        if (!profileInitialized_) {
          profileInitialized_ = true;
          const QString id = devices_->currentData().toString();
          if (profile_.load(calibrationPath(id), id, width, height))
            status_->setText(QString("Loaded %1 calibration station(s)").arg(profile_.stationCount()));
        }
      }
      if (mode_ == Mode::None || !capture_) return;
      if (!capture_->add(depth)) { status_->setText("Invalid depth capture"); mode_ = Mode::None; return; }
      progress_->setValue(capture_->frames());
      if (capture_->frames() < 30) return;
      CalibrationReport report;
      if (mode_ == Mode::Station) {
        if (profile_.addStation(*capture_, tape_->value(), report)) {
          validated_ = false;
          status_->setText(QString("Station %1 staged · training RMS %2 → %3 mm. Validate separately.")
              .arg(profile_.stationCount()).arg(report.beforeRmsMm, 0, 'f', 1)
              .arg(report.afterRmsMm, 0, 'f', 1));
        } else status_->setText("Wall capture failed: check coverage and distance");
      } else {
        report = profile_.benchmark(*capture_);
        validated_ = report.valid && report.afterRmsMm < report.beforeRmsMm;
        status_->setText(report.valid ?
            QString("Validation RMS %1 → %2 mm · center %3 m · %4")
                .arg(report.beforeRmsMm, 0, 'f', 1).arg(report.afterRmsMm, 0, 'f', 1)
                .arg(report.centerDistanceM, 0, 'f', 2)
                .arg(validated_ ? "ready to save" : "correction did not improve") :
            "Validation failed: capture a flat wall with sufficient coverage");
      }
      mode_ = Mode::None; capture_.reset();
    });
    worker_->startLatest();
  }
  void stopDepth() {
    mode_ = Mode::None; capture_.reset();
    if (!worker_) return;
    worker_->requestInterruption(); worker_->wait(); delete worker_; worker_ = nullptr;
  }
  void beginCapture(Mode mode) {
    if (!worker_ || !worker_->isRunning() || !intrinsics_.valid()) {
      status_->setText("Start a metric depth stream first"); return;
    }
    if (mode == Mode::Validation && profile_.stationCount() == 0) {
      status_->setText("Capture at least one wall station first"); return;
    }
    mode_ = mode; capture_ = std::make_unique<CalibrationCapture>(intrinsics_);
    progress_->setValue(0);
    status_->setText(mode == Mode::Station ? "Capturing wall station…" : "Capturing validation wall…");
  }
  void saveProfile() {
    if (!validated_) { status_->setText("Validate the staged calibration before saving"); return; }
    const QString path = calibrationPath(devices_->currentData().toString());
    if (!QDir().mkpath(QFileInfo(path).absolutePath()) || !profile_.save(path)) {
      status_->setText("Could not save depth calibration"); return;
    }
    status_->setText("Saved calibration to " + path);
  }
  void resetProfile() {
    stopDepth();
    const QString id = devices_->currentData().toString();
    QFile::remove(calibrationPath(id));
    profile_ = DepthCalibrationProfile(id); intrinsics_ = {};
    profileInitialized_ = validated_ = false; progress_->setValue(0);
    status_->setText("Calibration reset for " + id);
  }
  std::vector<Device> found_;
  StreamWorker* worker_ = nullptr;
  QComboBox* devices_ = nullptr;
  QDoubleSpinBox* tape_ = nullptr;
  QProgressBar* progress_ = nullptr;
  QLabel* preview_ = nullptr;
  QLabel* status_ = nullptr;
  DepthIntrinsics intrinsics_;
  DepthCalibrationProfile profile_;
  std::unique_ptr<CalibrationCapture> capture_;
  Mode mode_ = Mode::None;
  bool profileInitialized_ = false, validated_ = false;
};

class SurveillanceWindow : public QMainWindow {
 public:
  SurveillanceWindow() {
    setWindowTitle("Kinect Remold — Surveillance");
    resize(960, 760);
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    auto* controls = new QHBoxLayout;
    armButton_ = new QPushButton("Arm motion recording");
    auto* manual = new QPushButton("Record now");
    auto* stop = new QPushButton("Stop recording");
    auto* refreshButton = new QPushButton("Refresh sensors");
    controls->addWidget(armButton_); controls->addWidget(manual);
    controls->addWidget(stop); controls->addStretch(); controls->addWidget(refreshButton);
    layout->addLayout(controls);
    status_ = new QLabel("Searching for cameras…"); layout->addWidget(status_);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    auto* list = new QWidget;
    cards_ = new QVBoxLayout(list);
    cards_->addStretch();
    scroll->setWidget(list);
    layout->addWidget(scroll, 1);
    setCentralWidget(central);
    connect(armButton_, &QPushButton::clicked, this, [this] {
      armed_ = !armed_;
      armedAt_ = QDateTime::currentMSecsSinceEpoch();
      armButton_->setText(armed_ ? "Disarm" : "Arm motion recording");
      for (auto& camera : cameras_) camera->detector.reset(24);
      if (!armed_ && recording_) stopEvent();
      else status_->setText(armed_ ? "Armed; waiting for motion" : "Disarmed");
    });
    connect(manual, &QPushButton::clicked, this, [this] { startEvent(); });
    connect(stop, &QPushButton::clicked, this, [this] { stopEvent(); });
    connect(refreshButton, &QPushButton::clicked, this, [this] { syncCameras(); });
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] {
      syncCameras();
      if (recording_ && QDateTime::currentMSecsSinceEpoch() - lastMotion_ > 60000) stopEvent();
    });
    timer->start(2000);
    syncCameras();
  }
  ~SurveillanceWindow() override {
    stopEvent();
    for (auto& camera : cameras_) stopCamera(*camera);
  }
  int checkLiveCameras() {
    if (cameras_.empty()) return 2;
    QElapsedTimer timeout; timeout.start();
    while (timeout.elapsed() < 10000) {
      QApplication::processEvents();
      for (const auto& camera : cameras_) if (camera->frames > 0) return 0;
      QThread::msleep(20);
    }
    return 3;
  }
  int checkRecording() {
    QTemporaryDir directory;
    if (!directory.isValid() || cameras_.empty()) return 2;
    recordingRoot_ = directory.path();
    QElapsedTimer timeout; timeout.start();
    while (timeout.elapsed() < 10000 && cameras_.front()->retention.size() < 3) {
      QApplication::processEvents(); QThread::msleep(20);
    }
    if (cameras_.front()->retention.size() < 3) return 3;
    startEvent();
    if (!recording_) return 4;
    const quint64 first = cameras_.front()->frames;
    timeout.restart();
    while (timeout.elapsed() < 10000 && cameras_.front()->frames < first + 12) {
      QApplication::processEvents(); QThread::msleep(20);
    }
    stopEvent();
    QDir root(directory.path());
    for (const QString& subdir : root.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
      QDir event(root.filePath(subdir));
      for (const QString& file : event.entryList({"*.avi"}, QDir::Files))
        if (QFileInfo(event.filePath(file)).size() > 256) return 0;
    }
    return 5;
  }
 private:
  struct Camera {
    struct RetainedFrame { qint64 epochMs = 0; QByteArray jpeg; };
    Device device;
    StreamWorker* worker = nullptr;
    QWidget* card = nullptr;
    QLabel* preview = nullptr;
    QLabel* status = nullptr;
    MotionDetector detector;
    MjpegAviWriter recorder;
    std::deque<RetainedFrame> retention;
    bool infrared = false;
    bool probing = false;
    int darkFrames = 0;
    int brightFrames = 0;
    int probeFrames = 0;
    qint64 lastModeMs = 0;
    qint64 lastStartMs = 0;
    qint64 lastRetainedMs = 0;
    quint64 frames = 0;
  };

  Camera* cameraById(const QString& id) {
    for (auto& camera : cameras_) if (camera->device.id == id) return camera.get();
    return nullptr;
  }
  void stopCamera(Camera& camera) {
    if (!camera.worker) return;
    camera.worker->requestInterruption();
    camera.worker->wait();
    delete camera.worker;
    camera.worker = nullptr;
  }
  void startCamera(Camera& camera) {
    stopCamera(camera);
    camera.detector.reset(12);
    camera.worker = new StreamWorker(camera.device, camera.infrared ? 2u : 1u, 0, this);
    camera.lastStartMs = QDateTime::currentMSecsSinceEpoch();
    const QString id = camera.device.id;
    connect(camera.worker, &StreamWorker::frameReady, this, [this, id](const QImage& image) {
      if (Camera* current = cameraById(id)) onFrame(*current, image);
    });
    connect(camera.worker, &StreamWorker::message, this, [this, id](const QString& text) {
      if (Camera* current = cameraById(id)) current->status->setText(text);
    });
    camera.worker->startLatest();
  }
  void switchMode(const QString& id, bool infrared, bool probing = false) {
    QTimer::singleShot(0, this, [this, id, infrared, probing] {
      Camera* camera = cameraById(id);
      if (!camera || (camera->infrared == infrared && camera->probing == probing)) return;
      camera->infrared = infrared;
      camera->probing = probing;
      camera->darkFrames = camera->brightFrames = camera->probeFrames = 0;
      camera->lastModeMs = QDateTime::currentMSecsSinceEpoch();
      startCamera(*camera);
    });
  }
  void syncCameras() {
    std::vector<Device> found = discoverAt(sdkPath(false));
    auto one = discoverAt(sdkPath(true)); found.insert(found.end(), one.begin(), one.end());
    for (const Device& device : found) {
      if (device.state != QStringLiteral("Ready") || device.camera.isEmpty() || cameraById(device.id)) continue;
      auto camera = std::make_unique<Camera>();
      camera->device = device;
      camera->card = new QWidget;
      auto* layout = new QVBoxLayout(camera->card);
      layout->addWidget(new QLabel(device.label + " · " + device.id));
      camera->preview = new QLabel("Waiting for video");
      camera->preview->setMinimumSize(320, 240);
      camera->preview->setAlignment(Qt::AlignCenter);
      camera->preview->setStyleSheet("background:#151b22;color:#e5e9ef;");
      layout->addWidget(camera->preview);
      camera->status = new QLabel("Starting…");
      layout->addWidget(camera->status);
      cards_->insertWidget(cards_->count() - 1, camera->card);
      cameras_.push_back(std::move(camera));
      startCamera(*cameras_.back());
    }
    for (auto it = cameras_.begin(); it != cameras_.end();) {
      const auto present = std::find_if(found.begin(), found.end(), [&](const Device& device) {
        return device.id == (*it)->device.id && device.state == QStringLiteral("Ready") &&
            device.camera == (*it)->device.camera;
      });
      if (present == found.end()) {
        stopCamera(**it);
        if ((*it)->recorder.active()) (*it)->recorder.finish();
        delete (*it)->card;
        it = cameras_.erase(it);
      } else {
        if ((*it)->worker && !(*it)->worker->isRunning() &&
            QDateTime::currentMSecsSinceEpoch() - (*it)->lastStartMs > 2000) startCamera(**it);
        ++it;
      }
    }
    if (!recording_) status_->setText(QString("%1 camera(s) · %2")
        .arg(cameras_.size()).arg(armed_ ? "armed" : "disarmed"));
  }
  void onFrame(Camera& camera, const QImage& image) {
    ++camera.frames;
    camera.preview->setPixmap(QPixmap::fromImage(image).scaled(640, 360,
        Qt::KeepAspectRatio, Qt::SmoothTransformation));
    const MotionSample motion = camera.detector.process(image);
    camera.status->setText(QString("%1 · motion %2%")
        .arg(camera.infrared ? "Infrared" : "RGB")
        .arg(int(motion.score * 100)));
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - camera.lastRetainedMs >= 200) {
      camera.lastRetainedMs = now;
      QImage retained = image.scaled(320, 240, Qt::IgnoreAspectRatio,
                                    Qt::SmoothTransformation).convertToFormat(QImage::Format_RGB32);
      {
        QPainter painter(&retained);
        painter.setPen(Qt::white);
        painter.fillRect(0, 0, retained.width(), 28, QColor(0, 0, 0, 160));
        painter.drawText(8, 19, QDateTime::fromMSecsSinceEpoch(now).toString("yyyy-MM-dd HH:mm:ss") +
                               " · " + camera.device.label);
      }
      QByteArray jpeg;
      QBuffer buffer(&jpeg);
      buffer.open(QIODevice::WriteOnly);
      if (retained.save(&buffer, "JPEG", 40)) {
        camera.retention.push_back({now, jpeg});
        while (!camera.retention.empty() && now - camera.retention.front().epochMs > 600000)
          camera.retention.pop_front();
        if (recording_ && camera.recorder.active() && !camera.recorder.append(jpeg))
          camera.status->setText(camera.recorder.error());
      }
    }
    if (motion.moving) lastMotion_ = now;
    if (armed_ && motion.triggered && now - armedAt_ >= 2500 && !recording_) startEvent();
    if (!camera.infrared) {
      if (camera.probing) {
        ++camera.probeFrames;
        camera.brightFrames = motion.meanLuma >= 0.34f ? camera.brightFrames + 1 : 0;
        if (camera.brightFrames >= 4) camera.probing = false;
        else if (camera.probeFrames >= 6) switchMode(camera.device.id, true);
      } else {
        camera.darkFrames = motion.meanLuma <= 0.24f ? camera.darkFrames + 1 : 0;
        if (camera.darkFrames >= 8 && now - camera.lastModeMs >= 60000)
          switchMode(camera.device.id, true);
      }
    } else if (now - camera.lastModeMs >= 60000) {
      switchMode(camera.device.id, false, true);
    }
  }
  void startEvent() {
    if (recording_ || cameras_.empty()) return;
    const QString root = (recordingRoot_.isEmpty() ?
        QStandardPaths::writableLocation(QStandardPaths::MoviesLocation) + "/Kinect Surveillance" :
        recordingRoot_) +
        "/" + QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss");
    if (!QDir().mkpath(root)) { status_->setText("Could not create recording folder"); return; }
    int started = 0;
    for (auto& camera : cameras_) {
      QString safe = camera->device.id;
      safe.replace(QRegularExpression("[^A-Za-z0-9_-]"), "_");
      if (camera->recorder.start(root + "/" + safe + ".avi", 320, 240, 5)) {
        bool valid = true;
        for (const Camera::RetainedFrame& frame : camera->retention) {
          if (frame.epochMs < QDateTime::currentMSecsSinceEpoch() - 60000) continue;
          if (!camera->recorder.append(frame.jpeg)) { valid = false; break; }
        }
        if (valid) ++started;
        else {
          camera->status->setText(camera->recorder.error());
          camera->recorder.finish();
        }
      } else camera->status->setText(camera->recorder.error());
    }
    recording_ = started > 0;
    lastMotion_ = QDateTime::currentMSecsSinceEpoch();
    status_->setText(recording_ ? QString("Recording %1 camera(s) to %2").arg(started).arg(root) :
                                   "No camera recorder could start");
  }
  void stopEvent() {
    if (!recording_) return;
    recording_ = false;
    for (auto& camera : cameras_) {
      if (camera->recorder.active() && !camera->recorder.finish())
        camera->status->setText(camera->recorder.error());
    }
    status_->setText(armed_ ? "Armed; waiting for motion" : "Recording stopped");
  }
  std::vector<std::unique_ptr<Camera>> cameras_;
  QVBoxLayout* cards_ = nullptr;
  QLabel* status_ = nullptr;
  QPushButton* armButton_ = nullptr;
  bool armed_ = false, recording_ = false;
  qint64 armedAt_ = 0, lastMotion_ = 0;
  QString recordingRoot_;
};

class ScannerFusionWorker : public QThread {
  Q_OBJECT
 public:
  explicit ScannerFusionWorker(QString deviceId, bool turntable, QObject* parent = nullptr)
      : QThread(parent), deviceId_(std::move(deviceId)), turntable_(turntable),
        volume_(192,.008f,.032f) {}
  void submit(QByteArray depth, DepthIntrinsics camera,QImage color={}) {
    QMutexLocker lock(&mutex_);
    pendingDepth_ = std::move(depth); pendingCamera_ = std::move(camera);
    pendingColor_=std::move(color);
    hasPending_ = true; ready_.wakeOne();
  }
  void stop() {
    requestInterruption(); ready_.wakeAll(); wait();
  }
  const TsdfVolume& volume() const { return volume_; } // Read after stop().
 signals:
  void progress(int frames, int voxels, float sweepDegrees);
  void failed(QString reason);
 protected:
  void run() override {
    DepthCalibrationProfile calibration(deviceId_);
    bool calibrationChecked = false, centered = false;
    float targetDistanceM = 0;
    QByteArray referenceDepth;
    ScannerPose worldPose;
    ScannerIcp tracker;
    float previousYaw=0,sweepDegrees=0;
    ScannerSupportPlane support;
    bool supportChecked=false;
    while (!isInterruptionRequested()) {
      QByteArray depth; DepthIntrinsics camera;QImage color;
      {
        QMutexLocker lock(&mutex_);
        if (!hasPending_) ready_.wait(&mutex_, 500);
        if (isInterruptionRequested()) break;
        if (!hasPending_) continue;
        depth = std::move(pendingDepth_); camera = std::move(pendingCamera_);
        color=std::move(pendingColor_);
        hasPending_ = false;
      }
      if (!camera.valid() || depth.size() != qsizetype(camera.width)*camera.height*2) continue;
      if (!calibrationChecked) {
        calibrationChecked = true;
        calibration.load(calibrationPath(deviceId_), deviceId_, camera.width, camera.height);
      }
      if (!centered) {
        std::vector<quint16> samples;
        for (int y=camera.height/3;y<camera.height*2/3;y+=8)
          for (int x=camera.width/3;x<camera.width*2/3;x+=8) {
            quint16 mm=u16(depth.constData()+(y*camera.width+x)*2);
            if (mm>350 && mm<4000) samples.push_back(mm);
          }
        if (samples.size()<20) { emit failed("Point the sensor at a subject 0.35–4 m away"); continue; }
        std::nth_element(samples.begin(),samples.begin()+samples.size()/2,samples.end());
        targetDistanceM=samples[samples.size()/2]*.001f;
        volume_.reset({0,0,targetDistanceM}); centered = true;
      }
      if(turntable_) {
        for(int y=0;y<camera.height;++y) for(int x=0;x<camera.width;++x) {
          const int offset=(y*camera.width+x)*2;
          const quint16 mm=u16(depth.constData()+offset);
          if(x<camera.width/10||x>=camera.width*9/10||
             y<camera.height/10||y>=camera.height*9/10||
             mm<std::max(350.0f,(targetDistanceM-.5f)*1000)||
             mm>(targetDistanceM+.5f)*1000) {
            depth[offset]=0;depth[offset+1]=0;
          }
        }
        if(!supportChecked) {
          support=ScannerSegmentation::findSupport(depth,camera,&calibration);
          supportChecked=true;
        }
        support.removeSupport(depth,camera,&calibration);
      }
      ScannerPose candidatePose=worldPose;
      if (!referenceDepth.isEmpty()) {
        const ScannerIcpResult tracking=tracker.align(depth,referenceDepth,camera,&calibration,
                                                      turntable_);
        if (!tracking.accepted) {
          emit failed(QString("Tracking lost: %1 matches, %2 mm RMS; frame skipped")
              .arg(tracking.matches).arg(tracking.rmsM*1000,0,'f',1));
          continue;
        }
        candidatePose=worldPose.compose(tracking.currentToReference);
      }
      if (volume_.observedVoxels()>0) {
        QVector3D low(100,100,100),high(-100,-100,-100);
        int bounds=0;
        for(int y=camera.height/8;y<camera.height*7/8;y+=12)
          for(int x=camera.width/8;x<camera.width*7/8;x+=12) {
            quint16 mm=u16(depth.constData()+(y*camera.width+x)*2);
            mm=calibration.correct(x,y,mm);
            const float z=mm*.001f;
            if(z<targetDistanceM-.5f||z>targetDistanceM+.5f) continue;
            double rx=0,ry=0;
            if(!camera.ray(x,y,rx,ry)) continue;
            const QVector3D p=candidatePose.world({float(rx)*z,float(ry)*z,z});
            low.setX(std::min(low.x(),p.x()));low.setY(std::min(low.y(),p.y()));
            low.setZ(std::min(low.z(),p.z()));
            high.setX(std::max(high.x(),p.x()));high.setY(std::max(high.y(),p.y()));
            high.setZ(std::max(high.z(),p.z()));++bounds;
          }
        if(bounds>20) volume_.expandToInclude(low,high);
      }
      if (volume_.integrate(depth,camera,&calibration,candidatePose,3,
                            color.isNull()?nullptr:&color)) {
        if(turntable_) {
          const float yaw=std::atan2(candidatePose.m[2],candidatePose.m[0]);
          sweepDegrees+=std::abs(std::remainder(yaw-previousYaw,6.28318530718f))*57.2957795f;
          previousYaw=yaw;
        }
        worldPose=candidatePose; referenceDepth=std::move(depth);
        emit progress(volume_.integratedFrames(),volume_.observedVoxels(),sweepDegrees);
      }
    }
  }
 private:
  QString deviceId_;
  bool turntable_ = false;
  QMutex mutex_;
  QWaitCondition ready_;
  QByteArray pendingDepth_;
  DepthIntrinsics pendingCamera_;
  QImage pendingColor_;
  bool hasPending_ = false;
  TsdfVolume volume_;
};

class ScannerMeshPreview : public QWidget {
 public:
  explicit ScannerMeshPreview(QWidget* parent=nullptr) : QWidget(parent) {
    setMinimumSize(360,320);
  }
  void setMesh(const std::vector<ScannerTriangle>& triangles) {
    points_.clear();
    if(triangles.empty()) {update();return;}
    const size_t stride=std::max<size_t>(1,triangles.size()/9000);
    QVector3D low(100,100,100),high(-100,-100,-100);
    for(size_t i=0;i<triangles.size();i+=stride) {
      const QVector3D p=(triangles[i].a+triangles[i].b+triangles[i].c)/3;
      const QRgb rgb=triangles[i].ca;
      points_.push_back({p,qRed(rgb)==180&&qGreen(rgb)==180&&qBlue(rgb)==180
                           ?QColor(85,221,179):QColor(rgb)});
      low.setX(std::min(low.x(),p.x()));low.setY(std::min(low.y(),p.y()));
      low.setZ(std::min(low.z(),p.z()));
      high.setX(std::max(high.x(),p.x()));high.setY(std::max(high.y(),p.y()));
      high.setZ(std::max(high.z(),p.z()));
    }
    center_=(low+high)*.5f;
    span_=std::max({high.x()-low.x(),high.y()-low.y(),high.z()-low.z(),.1f});
    update();
  }
 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);painter.fillRect(rect(),QColor(15,21,28));
    painter.setRenderHint(QPainter::Antialiasing,false);
    if(points_.empty()) {
      painter.setPen(QColor(220,228,237));
      painter.drawText(rect(),Qt::AlignCenter,"Stop capture to inspect the fused mesh.");
      return;
    }
    const float cy=std::cos(yaw_),sy=std::sin(yaw_);
    const float cp=std::cos(pitch_),sp=std::sin(pitch_);
    const float scale=std::min(width(),height())*.78f/span_;
    for(const auto& sample:points_) {
      painter.setPen(sample.color);
      const QVector3D p=sample.point-center_;
      const float x=cy*p.x()+sy*p.z();
      const float z=-sy*p.x()+cy*p.z();
      const float y=cp*p.y()-sp*z;
      const float depth=sp*p.y()+cp*z;
      const float perspective=1.0f/(1.0f+depth/(span_*2.5f));
      const int sx=int(width()*.5f+x*scale*perspective);
      const int sy=int(height()*.5f+y*scale*perspective);
      if(sx>=0&&sy>=0&&sx<width()&&sy<height()) painter.drawPoint(sx,sy);
    }
    painter.setPen(QColor(210,220,230));
    painter.drawText(12,22,QString("%1 surface samples · drag to orbit").arg(points_.size()));
  }
  void mousePressEvent(QMouseEvent* event) override { lastMouse_=event->pos(); }
  void mouseMoveEvent(QMouseEvent* event) override {
    if(!(event->buttons()&Qt::LeftButton)) return;
    const QPoint delta=event->pos()-lastMouse_;lastMouse_=event->pos();
    yaw_+=delta.x()*.008f;
    pitch_=std::clamp(pitch_+delta.y()*.008f,-1.45f,1.45f);
    update();
  }
 private:
  struct PreviewPoint {QVector3D point;QColor color;};
  std::vector<PreviewPoint> points_;
  QVector3D center_;
  float span_=1,yaw_=0.25f,pitch_=-.2f;
  QPoint lastMouse_;
};

class ScannerWindow : public QMainWindow {
 public:
  ScannerWindow() {
    setWindowTitle("Kinect Remold — 3D Scanner"); resize(900,700);
    auto* central=new QWidget(this); auto* layout=new QVBoxLayout(central);
    auto* controls=new QHBoxLayout;
    devices_=new QComboBox;
    mode_=new QComboBox;mode_->addItems({"Moving Kinect", "Turntable"});
    auto* refreshButton=new QPushButton("Refresh");
    auto* startButton=new QPushButton("Start TSDF capture");
    auto* stopButton=new QPushButton("Stop");
    auto* exportButton=new QPushButton("Export mesh");
    controls->addWidget(devices_,1); controls->addWidget(mode_); controls->addWidget(refreshButton);
    controls->addWidget(startButton); controls->addWidget(stopButton);
    controls->addWidget(exportButton); layout->addLayout(controls);
    auto* views=new QHBoxLayout;
    preview_=new QLabel("Select a metric depth sensor to begin a TSDF capture.");
    preview_->setMinimumSize(360,320); preview_->setAlignment(Qt::AlignCenter);
    preview_->setStyleSheet("background:#151b22;color:#e5e9ef;");
    meshPreview_=new ScannerMeshPreview;
    views->addWidget(preview_,1);views->addWidget(meshPreview_,1);
    layout->addLayout(views,1);
    status_=new QLabel; layout->addWidget(status_); setCentralWidget(central);
    connect(refreshButton,&QPushButton::clicked,this,[this]{refresh();});
    connect(startButton,&QPushButton::clicked,this,[this]{start();});
    connect(stopButton,&QPushButton::clicked,this,[this]{stop();showMeshPreview();});
    connect(exportButton,&QPushButton::clicked,this,[this]{exportMesh();});
    refresh();
  }
  ~ScannerWindow() override { stop(); delete fusion_; }
  int checkLiveFusion(int targetFrames = 3, bool turntable = false) {
    if(found_.empty()) return 2;
    mode_->setCurrentIndex(turntable?1:0);
    start();
    QElapsedTimer timeout; timeout.start();
    while(timeout.elapsed()<60000 && lastFrames_<targetFrames) {
      QApplication::processEvents();
      QThread::msleep(20);
    }
    stop();
    if(lastFrames_<targetFrames || !fusion_) return 3;
    const auto triangles=fusion_->volume().mesh(2);
    const int colored=int(std::count_if(triangles.begin(),triangles.end(),[](const ScannerTriangle& t){
      return qRed(t.ca)!=180||qGreen(t.ca)!=180||qBlue(t.ca)!=180;
    }));
    fprintf(stderr,"TSDF: %d frames, %d rejected, %d voxels, %zu triangles, %d colored\n",lastFrames_,
            rejectedFrames_,fusion_->volume().observedVoxels(),triangles.size(),colored);
    return triangles.size()>100&&(targetFrames<10||colored>100)?0:4;
  }
 private:
  void refresh() {
    if (stream_) return;
    const QString selected=devices_->currentData().toString();
    found_=discoverAt(sdkPath(false));
    auto one=discoverAt(sdkPath(true)); found_.insert(found_.end(),one.begin(),one.end());
    found_.erase(std::remove_if(found_.begin(),found_.end(),[](const Device& d){
      return d.camera.isEmpty() || d.state!="Ready";
    }),found_.end());
    devices_->clear(); for(const auto& d:found_) devices_->addItem(d.label,d.id);
    const int index=devices_->findData(selected); if(index>=0) devices_->setCurrentIndex(index);
    status_->setText(QString("%1 depth camera(s) available").arg(found_.size()));
  }
  void start() {
    stop();
    delete fusion_; fusion_=nullptr;
    lastFrames_=rejectedFrames_=0;
    const int index=devices_->currentIndex();
    if(index<0 || index>=int(found_.size())) return;
    const Device d=found_[size_t(index)];
    const bool turntable=mode_->currentIndex()==1;
    fusion_=new ScannerFusionWorker(d.id,turntable,this);
    connect(fusion_,&ScannerFusionWorker::progress,this,[this,turntable](int frames,int voxels,float sweep){
      lastFrames_=frames;
      status_->setText(turntable?QString("TSDF: %1 frames, %2 voxels, %3° sweep")
          .arg(frames).arg(voxels).arg(sweep,0,'f',1):
          QString("TSDF: %1 frames, %2 observed voxels").arg(frames).arg(voxels));
    });
    connect(fusion_,&ScannerFusionWorker::failed,this,[this](const QString& reason){
      if(reason.startsWith("Tracking lost")) ++rejectedFrames_;
      status_->setText(reason);
    });
    fusion_->start();
    const bool colorCapture=d.generation=="xbox-one";
    stream_=new StreamWorker(d,colorCapture?5u:4u,0,this);
    auto showPreview=[this](const QImage& image){
      preview_->setPixmap(QPixmap::fromImage(image).scaled(preview_->size(),Qt::KeepAspectRatio,
                                                          Qt::SmoothTransformation));
    };
    if(colorCapture)connect(stream_,&StreamWorker::rgbReady,this,
        [this,showPreview](const QImage& image,quint64 tick){
          latestRgb_=image;latestRgbTick_=tick;latestRgbTime_.restart();showPreview(image);
        });
    else connect(stream_,&StreamWorker::frameReady,this,showPreview);
    connect(stream_,&StreamWorker::depthStamp,this,[this](quint64 tick){latestDepthTick_=tick;});
    connect(stream_,&StreamWorker::depthReady,this,
            [this](const QByteArray& depth,int w,int h,double fx,double fy,double cx,double cy,
                   const QByteArray& extension) {
      const quint64 skew=latestRgbTick_>latestDepthTick_?
          latestRgbTick_-latestDepthTick_:latestDepthTick_-latestRgbTick_;
      if(fusion_)fusion_->submit(depth,{w,h,fx,fy,cx,cy,extension},
          latestRgbTime_.isValid()&&latestRgbTime_.elapsed()<150&&skew<=100?latestRgb_:QImage{});
    });
    connect(stream_,&StreamWorker::message,status_,&QLabel::setText);
    stream_->startLatest();
  }
  void stop() {
    if(stream_) { stream_->requestInterruption(); stream_->wait(); delete stream_; stream_=nullptr; }
    if(fusion_) fusion_->stop();
  }
  void showMeshPreview() {
    if(fusion_ && fusion_->volume().integratedFrames()>0)
      meshPreview_->setMesh(fusion_->volume().mesh(2));
  }
  void exportMesh() {
    stop();
    if(!fusion_ || fusion_->volume().integratedFrames()==0) { status_->setText("No fused surface to export"); return; }
    const auto triangles=fusion_->volume().mesh(2);
    meshPreview_->setMesh(triangles);
    if(triangles.empty()) { status_->setText("No mature TSDF surface yet; capture more frames"); return; }
    const QString folder=QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString suggested=folder+"/Kinect-scan-"+
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")+".ply";
    const QString path=QFileDialog::getSaveFileName(this,"Export fused mesh",suggested,
        "PLY mesh (*.ply);;STL mesh (*.stl);;Wavefront OBJ mesh (*.obj)");
    if(path.isEmpty()) return;
    QString error;
    status_->setText(ScannerMeshWriter::write(path,triangles,error)?
        QString("Exported %1 TSDF triangles").arg(triangles.size()):error);
  }
  std::vector<Device> found_;
  QComboBox* devices_=nullptr; QComboBox* mode_=nullptr;
  QLabel* preview_=nullptr; QLabel* status_=nullptr;
  ScannerMeshPreview* meshPreview_=nullptr;
  StreamWorker* stream_=nullptr; ScannerFusionWorker* fusion_=nullptr;
  int lastFrames_=0;
  int rejectedFrames_=0;
  QImage latestRgb_;QElapsedTimer latestRgbTime_;
  quint64 latestRgbTick_=0,latestDepthTick_=0;
};

#ifdef REMOLD_HAS_ONNX
class MocapStageView : public QWidget {
 public:
  enum class View { Overlay, Split, ThreeD };
  explicit MocapStageView(QWidget* parent=nullptr):QWidget(parent) {
    setMinimumSize(640,420);
  }
  void setFrame(const QImage& image) {frame_=image;update();}
  void setPose(const QByteArray& json) {pose_=QJsonDocument::fromJson(json).object();update();}
  void setView(View view) {view_=view;update();}
  void setFaceSurface(bool enabled) {faceSurface_=enabled;update();}
 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);p.fillRect(rect(),QColor(13,20,30));
    p.setRenderHint(QPainter::Antialiasing,true);
    if(view_!=View::ThreeD) {
      const QRect area=view_==View::Split?QRect(0,0,width()/2,height()):rect();
      drawVideo(p,area);
    }
    if(view_!=View::Overlay) {
      const QRect area=view_==View::Split?QRect(width()/2,0,width()-width()/2,height()):rect();
      draw3D(p,area);
    }
  }
  void mousePressEvent(QMouseEvent* e) override {lastMouse_=e->pos();}
  void mouseMoveEvent(QMouseEvent* e) override {
    if(!(e->buttons()&Qt::LeftButton)||view_==View::Overlay)return;
    const QPoint d=e->pos()-lastMouse_;lastMouse_=e->pos();
    yaw_+=d.x()*.008f;pitch_=std::clamp(pitch_+d.y()*.008f,-1.3f,1.3f);update();
  }
 private:
  static QJsonObject joints(const QJsonObject& root) {
    const QJsonArray bodies=root.value("bodies").toArray();
    return bodies.isEmpty()?QJsonObject{}:bodies.first().toObject().value("joints").toObject();
  }
  static QVector3D position(const QJsonObject& joint) {
    const QJsonArray a=joint.value("p").toArray();
    return a.size()==3?QVector3D(float(a[0].toDouble()),float(a[1].toDouble()),
                                  float(a[2].toDouble())):QVector3D{};
  }
  static QPointF imagePoint(const QJsonObject& joint,const QRectF& imageRect) {
    const QJsonArray uv=joint.value("uv").toArray();
    if(uv.size()!=2)return {-1,-1};
    return {imageRect.x()+uv[0].toDouble()*imageRect.width(),
            imageRect.y()+uv[1].toDouble()*imageRect.height()};
  }
  template<class Projection> static void drawBones(QPainter& p,const QJsonObject& j,Projection project) {
    static const std::pair<const char*,const char*> edges[]{
      {"shoulderL","shoulderR"},
      {"shoulderL","hipL"},{"shoulderR","hipR"},{"hipL","hipR"},
      {"shoulderL","elbowL"},{"elbowL","wristL"},
      {"shoulderR","elbowR"},{"elbowR","wristR"},
      {"hipL","kneeL"},{"kneeL","ankleL"},
      {"hipR","kneeR"},{"kneeR","ankleR"}};
    p.setPen(QPen(QColor(68,225,171),3,Qt::SolidLine,Qt::RoundCap));
    const QJsonObject head=j.value("head").toObject();
    const QJsonObject leftShoulder=j.value("shoulderL").toObject();
    const QJsonObject rightShoulder=j.value("shoulderR").toObject();
    if(!head.isEmpty()&&!leftShoulder.isEmpty()&&!rightShoulder.isEmpty()) {
      const QPointF top=project(head),left=project(leftShoulder),right=project(rightShoulder);
      if(top.x()>=0&&left.x()>=0&&right.x()>=0)
        p.drawLine(top,(left+right)*.5);
    }
    for(const auto& edge:edges) {
      const QJsonObject a=j.value(QLatin1String(edge.first)).toObject();
      const QJsonObject b=j.value(QLatin1String(edge.second)).toObject();
      if(a.isEmpty()||b.isEmpty())continue;
      const QPointF pa=project(a),pb=project(b);
      if(pa.x()>=0&&pb.x()>=0)p.drawLine(pa,pb);
    }
    p.setPen(QPen(QColor(248,215,95),2));p.setBrush(QColor(248,215,95));
    for(auto it=j.begin();it!=j.end();++it) {
      if(it.key().startsWith("hand")&& !it.key().endsWith("_0")&&
         !it.key().endsWith("_4")&&!it.key().endsWith("_8")&&
         !it.key().endsWith("_12")&&!it.key().endsWith("_16")&&
         !it.key().endsWith("_20"))continue;
      const QPointF point=project(it.value().toObject());
      if(point.x()>=0)p.drawEllipse(point,3,3);
    }
  }
  void drawFaceSurface(QPainter& p,const QRectF& target) const {
    if(!faceSurface_)return;
    const QJsonObject face=pose_.value("face").toObject();
    if(!face.value("tracked").toBool())return;
    const bool marginal=face.value("quality").toObject().value("state")=="yellow";
    const QJsonArray points=face.value("landmarks").toArray();
    if(points.size()!=478)return;
    auto point=[&](int index) {
      const QJsonArray a=points[index].toArray();
      return a.size()==3?QPointF(target.x()+a[0].toDouble()*target.width(),
          target.y()+a[1].toDouble()*target.height()):QPointF{};
    };
    // MediaPipe face oval, ordered around the visible boundary.
    static constexpr int oval[]{10,338,297,332,284,251,389,356,454,323,361,288,
        397,365,379,378,400,377,152,148,176,149,150,136,172,58,132,93,234,
        127,162,21,54,103,67,109};
    p.save();if(marginal)p.setOpacity(.35);p.setPen(Qt::NoPen);
    auto vertex=[&](int index) {
      const QJsonArray a=points[index].toArray();
      return QVector3D(float(target.x()+a[0].toDouble()*target.width()),
          float(target.y()+a[1].toDouble()*target.height()),
          float(a[2].toDouble()*target.width()));
    };
    for(const auto& triangle:remold::faceTriangles) {
      const QVector3D a=vertex(triangle[0]),b=vertex(triangle[1]),c=vertex(triangle[2]);
      const QVector3D normal=QVector3D::crossProduct(b-a,c-a);
      if(normal.z()>=-.01f)continue;
      const QVector3D facing=normal.normalized();
      const float light=std::clamp(.65f-facing.x()*.22f-facing.y()*.16f-facing.z()*.12f,.35f,1.f);
      p.setBrush(QColor(int(76*light),int(177*light),int(196*light),145));
      p.drawPolygon(QPolygonF{QPointF(a.x(),a.y()),QPointF(b.x(),b.y()),QPointF(c.x(),c.y())});
    }
    auto feature=[&](std::initializer_list<int> indices,const QColor& fill) {
      QPolygonF shape;for(int index:indices)shape<<point(index);
      p.setBrush(fill);p.drawPolygon(shape);
    };
    feature({33,160,158,133,153,144},QColor(15,35,48,195));
    feature({362,385,387,263,373,380},QColor(15,35,48,195));
    feature({61,40,37,0,267,270,291,321,314,17,84,91},QColor(97,37,62,200));
    p.setPen(QPen(QColor(163,244,251,180),1));p.setBrush(Qt::NoBrush);
    for(size_t i=0;i<std::size(oval);++i)
      p.drawLine(point(oval[i]),point(oval[(i+1)%std::size(oval)]));
    p.restore();
  }
  void drawVideo(QPainter& p,const QRect& area) {
    p.save();p.setClipRect(area);
    if(frame_.isNull()) {
      p.setPen(QColor(211,224,237));p.drawText(area,Qt::AlignCenter,"Start tracking to see live RGB-D footage.");
    } else {
      const QSize fitted=frame_.size().scaled(area.size(),Qt::KeepAspectRatio);
      const QRectF target(area.x()+(area.width()-fitted.width())*.5,
                          area.y()+(area.height()-fitted.height())*.5,
                          fitted.width(),fitted.height());
      p.drawImage(target,frame_);
      drawFaceSurface(p,target);
      drawBones(p,joints(pose_),[target](const QJsonObject& joint){return imagePoint(joint,target);});
    }
    p.setPen(QColor(230,239,247));p.drawText(area.adjusted(12,10,-10,-10),Qt::AlignTop,"VIDEO + 3D JOINTS");
    p.restore();
  }
  void draw3D(QPainter& p,const QRect& area) {
    p.save();p.setClipRect(area);p.fillRect(area,QColor(15,26,40));
    const QPointF center(area.center().x(),area.center().y()+area.height()*.14);
    const float scale=std::min(area.width(),area.height())*.26f;
    const float sy=std::sin(yaw_),cy=std::cos(yaw_),sp=std::sin(pitch_),cp=std::cos(pitch_);
    auto project=[&](QVector3D v){
      v-=QVector3D(0,0,2);
      const float x=cy*v.x()+sy*v.z(),z=-sy*v.x()+cy*v.z();
      const float y=cp*v.y()-sp*z,depth=sp*v.y()+cp*z;
      const float k=1.f/(1.f+depth*.18f);
      return QPointF(center.x()+x*scale*k,center.y()+y*scale*k);
    };
    p.setPen(QPen(QColor(51,78,100),1));
    for(int i=-5;i<=5;++i) {
      p.drawLine(project({float(i)*.4f,1,1}),project({float(i)*.4f,1,4}));
      p.drawLine(project({-2,1,2+float(i)*.4f}),project({2,1,2+float(i)*.4f}));
    }
    drawBones(p,joints(pose_),[&](const QJsonObject& joint){
      const QJsonArray a=joint.value("p").toArray();
      return a.size()==3?project(position(joint)):QPointF(-1,-1);
    });
    p.setPen(QColor(230,239,247));
    p.drawText(area.adjusted(12,10,-10,-10),Qt::AlignTop,"3D STAGE · DRAG TO ORBIT");
    p.restore();
  }
  QImage frame_;QJsonObject pose_;View view_=View::Split;bool faceSurface_=true;
  QPoint lastMouse_;float yaw_=-.3f,pitch_=-.2f;
};

class MocapPoseSmoother {
 public:
  void setStability(int percent) {stability_=std::clamp(percent,0,100)/100.;}
  void reset() {joints_.clear();device_.clear();face_=QJsonObject{};faceTime_=0;}
  QJsonObject filter(QJsonObject root) {
    const QString device=root.value("device").toString();
    if(device!=device_) {reset();device_=device;}
    const double time=root.value("t").toDouble();
    QJsonArray bodies=root.value("bodies").toArray();
    for(int bodyIndex=0;bodyIndex<bodies.size();++bodyIndex) {
      QJsonObject body=bodies[bodyIndex].toObject();
      QJsonObject observed=body.value("joints").toObject();
      const QStringList keys=observed.keys();
      for(const QString& name:keys) {
        QJsonObject joint=observed.value(name).toObject();
        const QJsonArray p=joint.value("p").toArray(),uv=joint.value("uv").toArray();
        if(p.size()!=3)continue;
        QVector3D measured(p[0].toDouble(),p[1].toDouble(),p[2].toDouble());
        if(!std::isfinite(measured.x())||!std::isfinite(measured.y())||
           !std::isfinite(measured.z()))continue;
        const QString key=QString::number(body.value("id").toInt())+":"+name;
        Joint& state=joints_[key];
        const double dt=time-state.time;
        QVector3D value=measured;
        double alpha=1;
        if(state.valid&&dt>.001&&dt<.25) {
          QVector3D delta=measured-state.position;
          const double maxStep=.05+2.*dt;
          const bool outlier=(measured-state.raw).length()>maxStep*2.;
          const double displacement=double(delta.length());
          const double confidence=std::clamp(joint.value("c").toDouble(),0.,1.);
          const double holdRadius=(.008+.012*stability_)*(1.5-.5*confidence);
          // Raw frame differences are dominated by tracker jitter while a
          // person is still. Measure sustained movement from the filtered
          // position and ignore motion inside the hold radius.
          const double motion=std::max(0.,displacement-holdRadius)/dt;
          state.velocity=state.velocity*.8f+QVector3D(float(motion),0,0)*.2f;
          if(delta.length()>maxStep)delta*=float(maxStep/delta.length());
          const double baseRate=1.5+5.*(1.-stability_);
          alpha=1.-std::exp(-dt*(baseRate+7.*std::min(double(state.velocity.x()),2.)));
          if(joint.value("c").toDouble()<.2)alpha*=.5;
          value=state.position+delta*float(outlier?.12*alpha:alpha);
          if(displacement<holdRadius)value=state.position;
        } else state.velocity={};
        joint["p"]=QJsonArray{value.x(),value.y(),value.z()};
        if(uv.size()==2) {
          const QPointF measuredUv(uv[0].toDouble(),uv[1].toDouble());
          const QPointF smoothUv=state.valid&&dt>.001&&dt<.25?
              state.uv+(measuredUv-state.uv)*alpha:measuredUv;
          joint["uv"]=QJsonArray{smoothUv.x(),smoothUv.y()};
          state.uv=smoothUv;
        }
        state.position=value;state.raw=measured;state.time=time;state.valid=true;
        observed[name]=joint;
      }
      body["joints"]=observed;bodies[bodyIndex]=body;
    }
    root["bodies"]=bodies;
    QJsonObject face=root.value("face").toObject();
    if(face.value("tracked").toBool()) {
      const double dt=time-faceTime_;
      if(!face_.isEmpty()&&dt>.001&&dt<.25) {
        const QJsonArray current=face.value("landmarks").toArray();
        const QJsonArray previous=face_.value("landmarks").toArray();
        if(current.size()==478&&previous.size()==478) {
          QJsonArray smoothed;
          const double alpha=1.-std::exp(-dt*(5.+7.*(1.-stability_)));
          for(int i=0;i<478;++i) {
            const QJsonArray now=current[i].toArray(),old=previous[i].toArray();
            if(now.size()!=3||old.size()!=3) {smoothed.append(now);continue;}
            QJsonArray point;
            for(int axis=0;axis<3;++axis) {
              const double delta=now[axis].toDouble()-old[axis].toDouble();
              // Follow deliberate motion promptly while damping pixel-scale
              // landmark noise on a still face.
              const double a=std::min(1.,alpha+std::max(0.,std::abs(delta)-.004)*10.);
              point.append(old[axis].toDouble()+delta*a);
            }
            smoothed.append(point);
          }
          face["landmarks"]=smoothed;
        }
        QJsonObject shapes=face.value("blendshapes").toObject();
        const QJsonObject oldShapes=face_.value("blendshapes").toObject();
        const double shapeAlpha=1.-std::exp(-dt*8.);
        for(auto it=shapes.begin();it!=shapes.end();++it) {
          const double now=it.value().toDouble();
          const double old=oldShapes.value(it.key()).toDouble(now);
          it.value()=old+(now-old)*std::min(1.,shapeAlpha+std::max(0.,std::abs(now-old)-.08));
        }
        face["blendshapes"]=shapes;
        QJsonObject head=face.value("head").toObject();
        const QJsonArray nowQ=head.value("q").toArray();
        const QJsonArray oldQ=face_.value("head").toObject().value("q").toArray();
        if(nowQ.size()==4&&oldQ.size()==4) {
          const QQuaternion now(nowQ[3].toDouble(),nowQ[0].toDouble(),
              nowQ[1].toDouble(),nowQ[2].toDouble());
          const QQuaternion old(oldQ[3].toDouble(),oldQ[0].toDouble(),
              oldQ[1].toDouble(),oldQ[2].toDouble());
          const double angular=2.*std::acos(std::clamp(double(std::abs(
              QQuaternion::dotProduct(now.normalized(),old.normalized()))),0.,1.));
          const double a=std::min(1.,1.-std::exp(-dt*(5.+std::min(angular/dt,5.)*1.5)));
          const QQuaternion q=QQuaternion::slerp(old.normalized(),now.normalized(),float(a));
          head["q"]=QJsonArray{q.x(),q.y(),q.z(),q.scalar()};
          face["head"]=head;
        }
      }
      face_=face;faceTime_=time;root["face"]=face;
    } else {face_=QJsonObject{};faceTime_=0;}
    return root;
  }
 private:
  struct Joint {QVector3D position,raw,velocity;QPointF uv;double time=0;bool valid=false;};
  QHash<QString,Joint> joints_;QString device_;QJsonObject face_;double faceTime_=0;double stability_=.7;
};

class MotionCaptureWindow : public QMainWindow {
 public:
  enum class GuidePhase {Idle,Position,Prepare,Capture,Complete};
  MotionCaptureWindow() {
    setWindowTitle("Kinect Remold — Motion Capture");resize(1280,980);
    auto* central=new QWidget(this);auto* layout=new QVBoxLayout(central);
    auto* top=new QHBoxLayout;
    devices_=new QComboBox;
    auto* refreshButton=new QPushButton("Refresh");
    auto* startButton=new QPushButton("Start tracking");
    auto* checkButton=new QPushButton("Run fit check");
    auto* stopButton=new QPushButton("Stop");
    top->addWidget(devices_,1);top->addWidget(refreshButton);
    top->addWidget(startButton);top->addWidget(checkButton);top->addWidget(stopButton);
    layout->addLayout(top);
    auto* sources=new QHBoxLayout;
    bodyEnabled_=new QCheckBox("Body + hands");bodyEnabled_->setChecked(true);
    faceEnabled_=new QCheckBox("Face");faceEnabled_->setChecked(true);
    sources->addWidget(new QLabel("CAPTURE SOURCES"));
    sources->addWidget(bodyEnabled_);sources->addWidget(faceEnabled_);
    sources->addWidget(new QLabel("Choose either source or both; changes apply when tracking starts."));
    sources->addWidget(new QLabel("Body stability"));
    stability_=new QSlider(Qt::Horizontal);stability_->setRange(0,100);
    stability_->setValue(70);stability_->setMaximumWidth(145);
    stability_->setToolTip("Left: more responsive. Right: steadier with more lag.");
    sources->addWidget(stability_);
    connect(stability_,&QSlider::valueChanged,this,[this](int value){
      smoother_.setStability(value);
    });
    sources->addStretch();layout->addLayout(sources);
    auto* performanceRow=new QHBoxLayout;
    performanceRow->addWidget(new QLabel("INFERENCE"));
    bodyProvider_=new QComboBox;handProvider_=new QComboBox;faceProvider_=new QComboBox;
#ifdef Q_OS_MACOS
    const QStringList providerChoices{"CPU","Core ML"};
#else
    const QStringList providerChoices{"CPU","CUDA"};
#endif
    bodyProvider_->addItems(providerChoices);handProvider_->addItems(providerChoices);
    faceProvider_->addItems(providerChoices);
    auto activeProvider=[](const char* key) {
      QByteArray value=qgetenv(key).toLower();
      if(value.isEmpty())value=qgetenv("REMOLD_ONNX_PROVIDER").toLower();
#ifdef Q_OS_MACOS
      return value=="coreml"?1:0;
#else
      return value=="cuda"?1:0;
#endif
    };
    bodyProvider_->setCurrentIndex(activeProvider("REMOLD_ONNX_BODY_PROVIDER"));
    handProvider_->setCurrentIndex(activeProvider("REMOLD_ONNX_HAND_PROVIDER"));
    faceProvider_->setCurrentIndex(activeProvider("REMOLD_ONNX_FACE_PROVIDER"));
    performanceRow->addWidget(new QLabel("Body"));performanceRow->addWidget(bodyProvider_);
    performanceRow->addWidget(new QLabel("Hands"));performanceRow->addWidget(handProvider_);
    performanceRow->addWidget(new QLabel("Face"));performanceRow->addWidget(faceProvider_);
    fitEnabled_=new QCheckBox("Articulated fit");fitEnabled_->setChecked(true);
    performanceRow->addWidget(fitEnabled_);
    connect(fitEnabled_,&QCheckBox::toggled,this,[this]{bodyFit_.reset();});
    profileEnabled_=new QCheckBox("Show timing");performanceRow->addWidget(profileEnabled_);
    profileEnabled_->setChecked(!qEnvironmentVariableIsEmpty("REMOLD_PROFILE_MOCAP"));
    auto* applyProvider=new QPushButton("Apply providers");
    applyProvider->setToolTip("Recreates the model sessions and restarts tracking. Stop recording first.");
    performanceRow->addWidget(applyProvider);
    performanceRow->addStretch();layout->addLayout(performanceRow);
    performance_=new QLabel("Timing off · provider changes restart tracking");
    layout->addWidget(performance_);
    connect(applyProvider,&QPushButton::clicked,this,[this]{
      if(recording_||bvh_.recording()) {
        performance_->setText("Stop recording before changing inference providers.");return;
      }
      auto choice=[](QComboBox* box){return box->currentIndex()==0?"cpu":
#ifdef Q_OS_MACOS
          "coreml";
#else
          "cuda";
#endif
      };
      qputenv("REMOLD_ONNX_BODY_PROVIDER",choice(bodyProvider_));
      qputenv("REMOLD_ONNX_HAND_PROVIDER",choice(handProvider_));
      qputenv("REMOLD_ONNX_FACE_PROVIDER",choice(faceProvider_));
      performance_->setText("Applying providers · tracking restarts");
      if(worker_)start();
    });
    connect(profileEnabled_,&QCheckBox::toggled,this,[this](bool on){
      if(on)qputenv("REMOLD_PROFILE_MOCAP","1");
      else qunsetenv("REMOLD_PROFILE_MOCAP");
      performance_->setText(on?"Timing starts after tracking restarts":"Timing off");
      if(worker_&&!recording_&&!bvh_.recording())start();
    });
    auto* qualityRow=new QHBoxLayout;
    quality_=new QLabel("NO BODY");quality_->setAlignment(Qt::AlignCenter);
    quality_->setMinimumSize(230,52);
    quality_->setStyleSheet("background:#a5232e;color:white;font-size:23px;font-weight:bold;border-radius:8px;");
    qualityDetail_=new QLabel("Start tracking and stand fully inside the camera view.");
    qualityDetail_->setStyleSheet("font-size:16px;font-weight:600;");
    qualityRow->addWidget(quality_);qualityRow->addWidget(qualityDetail_,1);
    layout->addLayout(qualityRow);
    guideBanner_=new QLabel;
    guideBanner_->setAlignment(Qt::AlignCenter);guideBanner_->setWordWrap(true);
    guideBanner_->setMinimumHeight(100);
    guideDetail_=new QLabel;guideDetail_->setAlignment(Qt::AlignCenter);
    guideDetail_->setWordWrap(true);guideDetail_->setStyleSheet("font-size:20px;font-weight:600;");
    layout->addWidget(guideBanner_);layout->addWidget(guideDetail_);
    guideBanner_->hide();guideDetail_->hide();
    auto* outputs=new QHBoxLayout;
    jsonEnabled_=new QCheckBox("JSON UDP");
    oscEnabled_=new QCheckBox("OSC UDP");
    vmcEnabled_=new QCheckBox("VMC");
    faceCapEnabled_=new QCheckBox("Face Cap OSC");
    webEnabled_=new QCheckBox("WebSocket");
    unityEnabled_=new QCheckBox("Unity JSON");
    mirrorEnabled_=new QCheckBox("Mirror avatar");
    jsonPort_=new QSpinBox;jsonPort_->setRange(1,65535);jsonPort_->setValue(39541);
    oscPort_=new QSpinBox;oscPort_->setRange(1,65535);oscPort_->setValue(39542);
    vmcPort_=new QSpinBox;vmcPort_->setRange(1,65535);vmcPort_->setValue(39539);
    faceCapPort_=new QSpinBox;faceCapPort_->setRange(1,65535);faceCapPort_->setValue(39543);
    webPort_=new QSpinBox;webPort_->setRange(1,65535);webPort_->setValue(39540);
    recordFormat_=new QComboBox;
    recordFormat_->addItems({"Compact clip","JSONL"});
    recordButton_=new QPushButton("Record clip");
    bvhButton_=new QPushButton("Record BVH");
    outputs->addWidget(jsonEnabled_);outputs->addWidget(jsonPort_);
    outputs->addWidget(oscEnabled_);outputs->addWidget(oscPort_);
    outputs->addWidget(vmcEnabled_);outputs->addWidget(vmcPort_);
    outputs->addWidget(webEnabled_);outputs->addWidget(webPort_);
    outputs->addWidget(unityEnabled_);outputs->addWidget(mirrorEnabled_);
    outputs->addWidget(recordFormat_);outputs->addWidget(recordButton_);outputs->addStretch();layout->addLayout(outputs);
    outputs->addWidget(bvhButton_);
    auto* faceOutputs=new QHBoxLayout;
    faceOutputs->addWidget(new QLabel("Face · 478 landmarks + 51 measured expressions"));
    faceOutputs->addWidget(faceCapEnabled_);faceOutputs->addWidget(faceCapPort_);
    faceOutputs->addWidget(new QLabel("VMC Face uses the VMC port; JSONL and WebSocket retain all landmarks"));
    faceOutputs->addStretch();layout->addLayout(faceOutputs);
    auto* faceMonitor=new QHBoxLayout;
    faceStatus_=new QLabel("Face: searching");faceMonitor->addWidget(faceStatus_);
    auto meter=[faceMonitor](const QString& name,QProgressBar*& bar){
      faceMonitor->addWidget(new QLabel(name));
      bar=new QProgressBar;bar->setRange(0,100);bar->setValue(0);
      bar->setMaximumWidth(110);bar->setTextVisible(false);faceMonitor->addWidget(bar);
    };
    meter("Jaw",jawMeter_);meter("Smile",smileMeter_);meter("Blink",blinkMeter_);
    faceMonitor->addStretch();layout->addLayout(faceMonitor);
    auto* calibrationPanel=new QWidget;
    auto* calibrationRow=new QVBoxLayout(calibrationPanel);
    calibrationList_=new QListWidget;
    calibrationList_->addItems({"1. Stand fully in view, arms relaxed",
        "2. Hold arms straight out (T pose)","3. Arms at sides",
        "4. Face forward, relaxed expression","5. Broad smile",
        "6. Mouth open","7. Close both eyes","8. Turn head left",
        "9. Turn head right"});
    calibrationList_->setMinimumHeight(185);calibrationList_->setCurrentRow(0);
    auto* guideButtons=new QVBoxLayout;
    auto* bodyGuideButton=new QPushButton("Guide body calibration");
    auto* faceGuideButton=new QPushButton("Guide face calibration");
    auto* cancelGuideButton=new QPushButton("Cancel guide");
    guideButtons->addWidget(bodyGuideButton);guideButtons->addWidget(faceGuideButton);
    guideButtons->addWidget(cancelGuideButton);
    auto* captureButton=new QPushButton("Capture selected pose");
    auto* resetCalibrationButton=new QPushButton("Reset calibration");
    calibrationStatus_=new QLabel("Select a pose, press Capture, and hold it until confirmed.");
    calibrationStatus_->setWordWrap(true);
    calibrationRow->addWidget(new QLabel("SETUP · FOLLOW THE LARGE INSTRUCTIONS"));
    calibrationRow->addLayout(guideButtons);
    calibrationRow->addWidget(new QLabel("1 beep starts · 2 beeps succeed · 3 beeps warn"));
    calibrationRow->addWidget(calibrationList_,1);
    calibrationRow->addWidget(captureButton);calibrationRow->addWidget(resetCalibrationButton);
    calibrationRow->addWidget(calibrationStatus_);
    connect(bodyGuideButton,&QPushButton::clicked,this,[this]{startGuide(false);});
    connect(faceGuideButton,&QPushButton::clicked,this,[this]{startGuide(true);});
    connect(cancelGuideButton,&QPushButton::clicked,this,[this]{cancelGuide();});
    guideTimer_.setInterval(1000);
    connect(&guideTimer_,&QTimer::timeout,this,[this]{
      if(guidePhase_!=GuidePhase::Prepare) {guideTimer_.stop();return;}
      if(--guideCountdown_>0) {
        guideDetail_->setText(QString("Get into the pose · capture starts in %1 seconds").arg(guideCountdown_));
      } else {
        guideTimer_.stop();guidePhase_=GuidePhase::Capture;calibrating_=true;
        QApplication::beep();
        guideDetail_->setText(QString("CAPTURING · %1/%2 valid frames")
            .arg(calibrationSamples_).arg(calibrationStep_==3?50:15));
      }
    });
    checkTimer_.setInterval(1000);
    connect(&checkTimer_,&QTimer::timeout,this,[this]{
      if(!checkPending_||!worker_||!worker_->isRunning()) {checkTimer_.stop();return;}
      if(--checkCountdown_>0) {
        guideDetail_->setText(QString("Get into position · check starts in %1 seconds").arg(checkCountdown_));
      } else {
        checkTimer_.stop();checkPending_=false;checkActive_=true;
        checkFrames_=checkBodyGood_=checkFaceGood_=0;
        QApplication::beep();
        setGuideText("CHECKING CAPTURE FIT","Hold your position · 0/60 frames");
      }
    });
    connect(captureButton,&QPushButton::clicked,this,[this]{
      cancelGuide(false);
      if(!worker_||!worker_->isRunning()) {
        calibrationStatus_->setText("Start tracking first, then capture a pose.");return;
      }
      calibrationStep_=calibrationList_->currentRow();calibrationSamples_=0;calibrationMisses_=0;
      calibrationWarned_=false;
      if((calibrationStep_<=2&&!activeBody_)||
         (calibrationStep_>=3&&!activeFace_)) {
        calibrationStatus_->setText("Enable the required capture source and restart tracking first.");return;
      }
      if(calibrationStep_==3) {
        calibrated_=false;calibrationSums_.clear();calibrationQuaternion_=QQuaternion();
      }
      calibrating_=false;
      calibrationStatus_->setText(QString("Get ready: %1 · capture starts in 2 seconds")
          .arg(calibrationList_->currentItem()->text()));
      const int generation=++calibrationGeneration_;
      QTimer::singleShot(2000,this,[this,generation]{
        if(generation!=calibrationGeneration_||!worker_||!worker_->isRunning())return;
        calibrating_=true;QApplication::beep();
        calibrationStatus_->setText(QString("Hold: %1 · 0/%2 valid frames")
            .arg(calibrationList_->item(calibrationStep_)->text())
            .arg(calibrationStep_==3?50:15));
      });
    });
    connect(resetCalibrationButton,&QPushButton::clicked,this,[this]{
      cancelGuide(false);
      ++calibrationGeneration_;calibrating_=false;calibrated_=false;calibrationSums_.clear();
      bodyCalibrated_=false;faceCalibrated_=false;
      for(int i=0;i<calibrationList_->count();++i)
        calibrationList_->item(i)->setForeground(QBrush());
      calibrationList_->setCurrentRow(0);
      calibrationStatus_->setText("Calibration reset. Select the first pose and press Capture.");
    });
    auto* viewRow=new QHBoxLayout;
    viewRow->addWidget(new QLabel("Stage"));
    viewMode_=new QComboBox;viewMode_->addItems({"Video overlay","Split video + 3D","3D stage"});
    viewMode_->setCurrentIndex(0);viewRow->addWidget(viewMode_);
    auto* faceSurface=new QCheckBox("Filled face surface");faceSurface->setChecked(true);
    viewRow->addWidget(faceSurface);
    viewRow->addStretch();
    viewRow->addWidget(new QLabel("Drag 3D stage to orbit"));
    layout->addLayout(viewRow);
    auto* workspace=new QHBoxLayout;
    stage_=new MocapStageView;stage_->setView(MocapStageView::View::Overlay);
    connect(faceSurface,&QCheckBox::toggled,stage_,&MocapStageView::setFaceSurface);
    workspace->addWidget(stage_,1);
    auto* sensorPanel=new QWidget;
    auto* sidebar=new QVBoxLayout(sensorPanel);
    sidebar->addWidget(new QLabel("SENSORS"));
    sensors_=new QTableWidget(0,4);
    sensors_->setHorizontalHeaderLabels({"Sensor","Source","Joints","Status"});
    sensors_->setSelectionBehavior(QAbstractItemView::SelectRows);
    sensors_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sensors_->horizontalHeader()->setSectionResizeMode(0,QHeaderView::Stretch);
    sensors_->setColumnWidth(1,55);sensors_->setColumnWidth(2,50);sensors_->setColumnWidth(3,95);
    sensors_->setMinimumWidth(360);sensors_->setMaximumWidth(440);
    sidebar->addWidget(sensors_,1);
    updateSensorRow("local","USB",0,"Waiting");
    sidebar->addWidget(new QLabel("LAN SENSOR HOSTS · MUTUAL TLS"));
    lanUrl_=new QLineEdit("wss://controller-host:39550/");
    lanUrl_->setPlaceholderText("wss://controller-host:39550/");
    sidebar->addWidget(lanUrl_);
    auto field=[sidebar](const QString& name,QLineEdit*& edit){
      auto* row=new QHBoxLayout;row->addWidget(new QLabel(name));
      edit=new QLineEdit;row->addWidget(edit,1);
      auto* browse=new QPushButton("…");row->addWidget(browse);
      QObject::connect(browse,&QPushButton::clicked,edit,[edit,name]{
        const QString path=QFileDialog::getOpenFileName(edit,"Select "+name);
        if(!path.isEmpty())edit->setText(path);
      });
      sidebar->addLayout(row);
    };
    field("Certificate",lanCert_);field("Private key",lanKey_);field("Trusted CA",lanCa_);
    auto* lanActions=new QHBoxLayout;
    lanPort_=new QSpinBox;lanPort_->setRange(1,65535);lanPort_->setValue(39550);
    auto* hostButton=new QPushButton("Listen as controller");
    auto* connectButton=new QPushButton("Join controller");
    auto* disconnectButton=new QPushButton("Disconnect");
    lanActions->addWidget(lanPort_);lanActions->addWidget(hostButton);
    lanActions->addWidget(connectButton);lanActions->addWidget(disconnectButton);
    sidebar->addLayout(lanActions);
    lanStatus_=new QLabel("Controller listens for authenticated sensor hosts. Each host selects a local Kinect, joins the controller, and starts tracking. Remote observations are preview-only until room calibration and clock sync are configured.");
    lanStatus_->setWordWrap(true);sidebar->addWidget(lanStatus_);
    sideTabs_=new QTabWidget;
    sideTabs_->setMinimumWidth(360);sideTabs_->setMaximumWidth(440);
    sideTabs_->addTab(calibrationPanel,"Calibrate");
    sideTabs_->addTab(sensorPanel,"Sensors & LAN");
    workspace->addWidget(sideTabs_);layout->addLayout(workspace,1);
    auto* timelineRow=new QHBoxLayout;
    timelineRow->addWidget(new QLabel("Pose timeline · last 60 s"));
    playButton_=new QPushButton("Play");liveButton_=new QPushButton("Go live");
    timeline_=new QSlider(Qt::Horizontal);timeline_->setRange(0,0);
    timelineLabel_=new QLabel("00:00 / LIVE");
    timelineRow->addWidget(playButton_);timelineRow->addWidget(liveButton_);
    timelineRow->addWidget(timeline_,1);timelineRow->addWidget(timelineLabel_);
    openClipButton_=new QPushButton("Open clip…");timelineRow->addWidget(openClipButton_);
    editButton_=new QPushButton("Edit recording…");timelineRow->addWidget(editButton_);
    layout->addLayout(timelineRow);
    storage_=new QLabel("Preview capped at 64 MiB · recordings stream to disk");
    layout->addWidget(storage_);
    status_=new QLabel;layout->addWidget(status_);setCentralWidget(central);
    connect(refreshButton,&QPushButton::clicked,this,[this]{refresh();});
    connect(startButton,&QPushButton::clicked,this,[this]{start();});
    connect(checkButton,&QPushButton::clicked,this,[this]{startFitCheck();});
    connect(stopButton,&QPushButton::clicked,this,[this]{stop();});
    connect(recordButton_,&QPushButton::clicked,this,[this]{toggleRecording();});
    connect(bvhButton_,&QPushButton::clicked,this,[this]{toggleBvh();});
    connect(editButton_,&QPushButton::clicked,this,[this]{editRecording();});
    connect(openClipButton_,&QPushButton::clicked,this,[this]{openClip();});
    connect(viewMode_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int index){
      stage_->setView(static_cast<MocapStageView::View>(index));
    });
    connect(timeline_,&QSlider::sliderMoved,this,[this](int index){
      live_=false;showTimeline(index);
    });
    connect(liveButton_,&QPushButton::clicked,this,[this]{
      if(clipFile_.isOpen()) {clipFile_.close();clipIndex_.reset();timeline_->setRange(0,0);}
      live_=true;playing_=false;playButton_->setText("Play");
      if(!history_.empty())showTimeline(int(history_.size())-1);
    });
    connect(playButton_,&QPushButton::clicked,this,[this]{
      if(history_.empty()&&!clipFile_.isOpen())return;
      playing_=!playing_;live_=false;playButton_->setText(playing_?"Pause":"Play");
      if(playing_&&timeline_->value()>=timeline_->maximum())timeline_->setValue(0);
    });
    playback_.setInterval(33);
    connect(&playback_,&QTimer::timeout,this,[this]{
      if(!playing_)return;
      if(timeline_->value()>=timeline_->maximum()) {
        playing_=false;playButton_->setText("Play");return;
      }
      showTimeline(timeline_->value()+1);
    });
    playback_.start();
    connect(hostButton,&QPushButton::clicked,this,[this]{
      if(!lan_.startHost(quint16(lanPort_->value()),lanCert_->text(),
                         lanKey_->text(),lanCa_->text()))lanStatus_->setText(lan_.error());
      else lanStatus_->setText(QString("Controller listening on wss://0.0.0.0:%1/")
                               .arg(lanPort_->value()));
    });
    connect(connectButton,&QPushButton::clicked,this,[this]{
      if(!lan_.connectPeer(QUrl(lanUrl_->text()),lanCert_->text(),
                           lanKey_->text(),lanCa_->text()))lanStatus_->setText(lan_.error());
      else lanStatus_->setText("Joining authenticated controller…");
    });
    connect(disconnectButton,&QPushButton::clicked,this,[this]{
      lan_.stop();lanStatus_->setText("LAN disconnected");
    });
    connect(&lan_,&LanPoseLink::failed,lanStatus_,&QLabel::setText);
    connect(&lan_,&LanPoseLink::peersChanged,this,[this](int count){
      lanStatus_->setText(QString("%1 authenticated LAN peer(s) · preview only").arg(count));
    });
    connect(&lan_,&LanPoseLink::remotePose,this,[this](const QString& sensor,const QByteArray& pose){
      const QJsonObject root=QJsonDocument::fromJson(pose).object();
      const int count=root.value("bodies").toArray().first().toObject().value("joints").toObject().size();
      remotePoses_[sensor]=pose;
      updateSensorRow(sensor,"LAN",count,"Preview only");
      if(selectedSensor_==sensor)stage_->setPose(pose);
    });
    connect(sensors_,&QTableWidget::cellClicked,this,[this](int row,int){
      selectedSensor_=sensors_->item(row,0)->text();
      if(selectedSensor_=="local") {
        if(!history_.empty())showTimeline(timeline_->value());
      } else {
        stage_->setFrame({});stage_->setPose(remotePoses_.value(selectedSensor_));
      }
    });
    refresh();
  }
  ~MotionCaptureWindow() override {stop();lan_.stop();if(editorThread_)editorThread_->wait();
    if(indexThread_)indexThread_->wait();}
  int checkLive(bool requireFace=false,bool requireFullBody=false,
                bool bodyOn=true,bool faceOn=true) {
    if(found_.empty()) return 2;
    bodyEnabled_->setChecked(bodyOn);faceEnabled_->setChecked(faceOn);
    fprintf(stderr,"Mocap sensor: %s\n",found_.front().label.toUtf8().constData());
    QUdpSocket jsonReceiver,oscReceiver,vmcReceiver,faceReceiver;
    if(!jsonReceiver.bind(QHostAddress::LocalHost,0)||
       !oscReceiver.bind(QHostAddress::LocalHost,0)||
       !vmcReceiver.bind(QHostAddress::LocalHost,0)||
       !faceReceiver.bind(QHostAddress::LocalHost,0)) return 3;
    jsonPort_->setValue(jsonReceiver.localPort());
    oscPort_->setValue(oscReceiver.localPort());
    vmcPort_->setValue(vmcReceiver.localPort());
    faceCapPort_->setValue(faceReceiver.localPort());
    jsonEnabled_->setChecked(true);oscEnabled_->setChecked(true);vmcEnabled_->setChecked(true);
    faceCapEnabled_->setChecked(true);
    QTemporaryDir directory;
    if(!directory.isValid()) return 4;
    const bool compact=QCoreApplication::arguments().contains("--check-live-compact");
    const QString path=directory.path()+(compact?"/poses.rmclip":"/poses.jsonl");
    const QString bvhPath=directory.path()+"/poses.bvh";
    start();
    if(!bvh_.start(bvhPath)) {stop();return 5;}
    file_.setFileName(path);
    if(!file_.open(QIODevice::WriteOnly)) {stop();return 5;}
    compactRecording_=compact;
    if(compact&&file_.write("RMC1",4)!=4){stop();return 5;}
    recorded_=0;recordedBytes_=compact?4:0;recording_=true;
    QElapsedTimer timeout;timeout.start();
    while(timeout.elapsed()<15000&&recorded_<((requireFace||requireFullBody)?150:10)) {
      QApplication::processEvents();QThread::msleep(20);
    }
    const int frames=recorded_;
    const QImage stageImage=stage_->grab().toImage();
    const QString solveCapture=qEnvironmentVariable("REMOLD_SOLVE_CAPTURE");
    if(!solveCapture.isEmpty())stageImage.save(solveCapture+".png");
    int stageJoints=0;
    for(int y=0;y<stageImage.height();++y)for(int x=0;x<stageImage.width();++x){
      const QColor c=stageImage.pixelColor(x,y);
      if(c.green()>c.red()*1.5&&c.green()>c.blue()*1.15)++stageJoints;
    }
    const bool timelineReady=timeline_->maximum()>=9;
    stop();
    const QString diagnostic=solveCapture.isEmpty()?
        qEnvironmentVariable("REMOLD_FACE_CAPTURE"):solveCapture;
    if(!diagnostic.isEmpty()) {
      QFile::remove(diagnostic);
      QFile::copy(path,diagnostic);
    }
    fprintf(stderr,"Mocap: %d frames, %d full-body frames, %d face frames, JSON %llu, OSC %llu, VMC %llu, largest JSON %d bytes, network %s\n",frames,fullBodyFrames_,faceFrames_,
            (unsigned long long)network_.jsonPackets(),
            (unsigned long long)network_.oscPackets(),
            (unsigned long long)network_.vmcPackets(),maxPoseBytes_,network_.error().toUtf8().constData());
    const QByteArray bvhData=[&]{QFile f(bvhPath);if(!f.open(QIODevice::ReadOnly))return QByteArray{};return f.readAll();}();
    QByteArray vmcPacket;
    while(vmcReceiver.hasPendingDatagrams()) {
      QByteArray packet(int(vmcReceiver.pendingDatagramSize()),0);
      vmcReceiver.readDatagram(packet.data(),packet.size());
      if(packet.contains("/VMC/Ext/Bone/Pos"))vmcPacket=packet;
    }
    QByteArray oscPacket;
    bool oscFrame=false,oscBone=false;
    while(oscReceiver.hasPendingDatagrams()) {
      QByteArray packet(int(oscReceiver.pendingDatagramSize()),0);
      oscReceiver.readDatagram(packet.data(),packet.size());
      oscFrame|=packet.contains("/synkinect/frame");
      oscBone|=packet.contains("/synkinect/bone");
      if(oscBone)oscPacket=packet;
    }
    const bool transport=frames>=10&&QFileInfo(path).size()>1000&&
        jsonReceiver.hasPendingDatagrams()&&oscFrame;
    bool compactReadable=true;
    if(compact) {
      MocapClipOptions options;options.source=path;
      options.output=directory.path()+"/roundtrip.jsonl";
      const auto roundtrip=MocapClipProcessor::process(options);
      compactReadable=roundtrip.error.isEmpty()&&roundtrip.frames==frames;
      fprintf(stderr,"Compact clip: %d decoded frames, %llu bytes, %s\n",
          roundtrip.frames,(unsigned long long)QFileInfo(path).size(),
          qPrintable(roundtrip.error));
    }
    QByteArray facePacket;
    while(faceReceiver.hasPendingDatagrams()) {
      QByteArray packet(int(faceReceiver.pendingDatagramSize()),0);
      faceReceiver.readDatagram(packet.data(),packet.size());
      if(packet.contains("/W"))facePacket=packet;
    }
    const bool rigOutput=network_.vmcPackets()==0||
        (bvhData.startsWith("HIERARCHY\n")&&bvhData.contains("MOTION\nFrames: ")&&
         bvhData.contains("JOINT LeftUpperArm")&&
         vmcPacket.contains("/VMC/Ext/Bone/Pos")&&vmcPacket.contains("Hips")&&
         oscBone);
    if(!(transport&&rigOutput&&timelineReady))
      fprintf(stderr,"Mocap gates: transport %d rig %d timeline %d stage pixels %d BVH %d VMC %d OSC bone %d\n",
          transport,rigOutput,timelineReady,stageJoints,
          bvhData.startsWith("HIERARCHY\n"),vmcPacket.contains("/VMC/Ext/Bone/Pos"),
          oscPacket.contains("/synkinect/bone"));
    if(network_.vmcPackets()==0) fprintf(stderr,"Mocap rig: no person tracked in this capture\n");
    return transport&&rigOutput&&timelineReady&&compactReadable&&
        (!requireFace||(faceFrames_>0&&facePacket.contains("/W")&&
                       network_.facePackets()>0))&&
        (!requireFullBody||(fullBodyFrames_>0&&network_.vmcPackets()>0))?0:6;
  }
  int checkGuidance() {
    QJsonObject joints;
    auto add=[&](const char* name,double x,double y){
      joints[QLatin1String(name)]=QJsonObject{{"p",QJsonArray{x,y,2.}},
          {"uv",QJsonArray{x,y}},{"c",.9}};
    };
    add("head",.5,.12);add("shoulderL",.42,.3);add("shoulderR",.58,.3);
    add("elbowL",.38,.43);add("elbowR",.62,.43);
    add("wristL",.35,.55);add("wristR",.65,.55);
    add("hipL",.45,.53);add("hipR",.55,.53);
    add("kneeL",.44,.69);add("kneeR",.56,.69);
    add("ankleL",.43,.85);add("ankleR",.57,.85);
    QJsonObject body{{"bodies",QJsonArray{QJsonObject{{"joints",joints}}}}};
    guideMode_=1;guidePhase_=GuidePhase::Position;
    if(!positionInstruction(body).isEmpty())return 2;
    for(int i=0;i<8;++i)updateGuidePosition(body);
    if(guidePhase_!=GuidePhase::Prepare||calibrationStep_!=0)return 3;
    guideTimer_.stop();
    QJsonObject closeJoints=joints;
    QJsonObject head=closeJoints.value("head").toObject();head["uv"]=QJsonArray{.5,.02};closeJoints["head"]=head;
    if(!positionInstruction(QJsonObject{{"bodies",QJsonArray{QJsonObject{{"joints",closeJoints}}}}})
            .contains("FARTHER"))return 4;
    QJsonObject fit=body;
    fit["quality"]=QJsonObject{{"state","green"}};
    fit["face"]=QJsonObject{{"tracked",true},{"confidence",.95}};
    checkActive_=true;checkFrames_=checkBodyGood_=checkFaceGood_=0;
    for(int i=0;i<60;++i)updateFitCheck(fit);
    if(!guideBanner_->text().contains("CALIBRATE BODY"))return 7;
    bodyCalibrated_=faceCalibrated_=true;
    checkActive_=true;checkFrames_=checkBodyGood_=checkFaceGood_=0;
    for(int i=0;i<60;++i)updateFitCheck(fit);
    if(guideBanner_->text()!="READY TO RECORD")return 8;
    guideMode_=2;guidePhase_=GuidePhase::Position;
    QJsonArray facePoints;
    for(int i=0;i<478;++i)facePoints.append(QJsonArray{i%2?.6:.4,i%4<2?.3:.6,0.});
    QJsonObject face{{"face",QJsonObject{{"tracked",true},{"confidence",.95},
                                       {"landmarks",facePoints}}}};
    if(!positionInstruction(face).isEmpty())return 5;
    for(int i=0;i<8;++i)updateGuidePosition(face);
    if(guidePhase_!=GuidePhase::Prepare||calibrationStep_!=3)return 6;
    guideTimer_.stop();
    auto turnedFace=[&](double degrees) {
      const QQuaternion turn=QQuaternion::fromAxisAndAngle(QVector3D(0,1,0),degrees);
      QJsonObject head{{"q",QJsonArray{turn.x(),turn.y(),turn.z(),turn.scalar()}}};
      QJsonObject observation=face.value("face").toObject();
      observation["head"]=head;
      return QJsonObject{{"face",observation}};
    };
    const auto left=turnedFace(30),right=turnedFace(-30);
    if(!calibrationMatches(left,7)||calibrationMatches(right,7) ||
       !calibrationMatches(right,8)||calibrationMatches(left,8))return 9;
    return 0;
  }
  int checkLiveFraming(bool faceMode) {
    bodyEnabled_->setChecked(!faceMode);faceEnabled_->setChecked(faceMode);start();
    if(!worker_)return 2;
    guideMode_=faceMode?2:1;guidePhase_=GuidePhase::Position;
    QElapsedTimer timer;timer.start();
    while(history_.size()<60&&timer.elapsed()<10000) {
      QApplication::processEvents();QThread::msleep(10);
    }
    const int frames=int(history_.size()),observed=faceMode?faceFrames_:fullBodyFrames_;
    const QString guidance=guideBanner_->text();
    fprintf(stderr,"%s framing: %d frames, %d observations, phase %d, instruction %s\n",
            faceMode?"Face":"Body",frames,observed,int(guidePhase_),
            guidance.toUtf8().constData());
    stop();return frames>=30&&observed>0?0:3;
  }
 private:
  void updateSensorRow(const QString& sensor,const QString& source,int joints,const QString& status) {
    int row=-1;
    for(int i=0;i<sensors_->rowCount();++i)
      if(sensors_->item(i,0)->text()==sensor) {row=i;break;}
    if(row<0) {row=sensors_->rowCount();sensors_->insertRow(row);
      for(int column=0;column<4;++column)sensors_->setItem(row,column,new QTableWidgetItem);}
    sensors_->item(row,0)->setText(sensor);sensors_->item(row,1)->setText(source);
    sensors_->item(row,2)->setText(QString::number(joints));
    sensors_->item(row,3)->setText(status);
  }
  void refresh() {
    if(worker_) return;
    const QString selected=devices_->currentData().toString();
    found_=discoverAt(sdkPath(false));
    auto one=discoverAt(sdkPath(true));found_.insert(found_.end(),one.begin(),one.end());
    found_.erase(std::remove_if(found_.begin(),found_.end(),[](const Device& d){
      return d.state!="Ready"||d.camera.isEmpty();
    }),found_.end());
    devices_->clear();for(const auto& d:found_) devices_->addItem(d.label,d.id);
    const int index=devices_->findData(selected);if(index>=0) devices_->setCurrentIndex(index);
    status_->setText(QString("%1 RGB-D camera(s) available").arg(found_.size()));
  }
  void start() {
    stop();
    const bool bodyOn=bodyEnabled_->isChecked(),faceOn=faceEnabled_->isChecked();
    if(!bodyOn&&!faceOn) {
      status_->setText("Select Body + hands, Face, or both before tracking.");return;
    }
    bvhButton_->setEnabled(bodyOn);
    activeBody_=bodyOn;activeFace_=faceOn;
    if(!bodyOn)calibrationList_->setCurrentRow(3);
    else if(!faceOn)calibrationList_->setCurrentRow(0);
    smoother_.reset();
    bodyFit_.reset();
    clipFile_.close();clipIndex_.reset();
    smoother_.setStability(stability_->value());
    history_.clear();historyBytes_=0;timeline_->setRange(0,0);live_=true;playing_=false;
    faceFrames_=0;fullBodyFrames_=0;maxPoseBytes_=0;
    playButton_->setText("Play");timelineLabel_->setText("00:00 / LIVE");
    const int index=devices_->currentIndex();
    if(index<0||index>=int(found_.size())) return;
    const Device d=found_[size_t(index)];
    network_.setJsonTarget(QHostAddress::LocalHost,quint16(jsonPort_->value()));
    network_.setOscTarget(QHostAddress::LocalHost,quint16(oscPort_->value()));
    network_.setVmcTarget(QHostAddress::LocalHost,quint16(vmcPort_->value()));
    network_.setFaceCapTarget(QHostAddress::LocalHost,quint16(faceCapPort_->value()));
    network_.setJsonEnabled(jsonEnabled_->isChecked());
    network_.setOscEnabled(oscEnabled_->isChecked());
    network_.setVmcEnabled(vmcEnabled_->isChecked());
    network_.setFaceCapEnabled(faceCapEnabled_->isChecked());
    if(webEnabled_->isChecked()&&!web_.start(quint16(webPort_->value()))) {
      status_->setText("WebSocket: "+web_.error());return;
    }
    worker_=new StreamWorker(d,bodyOn?5u:1u,bodyOn?(faceOn?5:7):6,this);
    connect(worker_,&StreamWorker::frameReady,this,[this](const QImage& image){
      lastFrame_=image;
      if(selectedSensor_=="local")stage_->setFrame(image);
    });
    connect(worker_,&StreamWorker::poseReady,this,[this](const QByteArray& rawPose){
      const QJsonObject rawRoot=QJsonDocument::fromJson(rawPose).object();
      QElapsedTimer fitTimer;
      const bool profileFit=!qEnvironmentVariableIsEmpty("REMOLD_PROFILE_MOCAP");
      if(profileFit)fitTimer.start();
      QJsonObject smoothed=smoother_.filter(rawRoot);
      QJsonObject root=fitEnabled_->isChecked()?bodyFit_.fit(std::move(smoothed)):std::move(smoothed);
      if(profileFit) {
        fitMs_+=fitTimer.nsecsElapsed()*1e-6;
        if(++fitFrames_%120==0)qInfo("Mocap filter + solve ms/frame (%d): %.3f",
            fitFrames_,fitMs_/fitFrames_);
        lastFitMs_=fitMs_/fitFrames_;
      }
      const bool tracked=root.value("tracked").toBool();
      if(root.value("face").toObject().value("tracked").toBool())++faceFrames_;
      QJsonObject face=root.value("face").toObject();
      updateGuidePosition(root);
      if(calibrating_&&calibrationMatches(root,calibrationStep_)) {
        calibrationMisses_=0;
        const QJsonObject shapes=face.value("blendshapes").toObject();
        if(calibrationStep_==3) {
          for(auto it=shapes.begin();it!=shapes.end();++it)
            calibrationSums_[it.key()]+=it.value().toDouble();
          const QJsonArray q=face.value("head").toObject().value("q").toArray();
          if(q.size()==4) {
          QQuaternion rotation(q[3].toDouble(),q[0].toDouble(),q[1].toDouble(),q[2].toDouble());
          if(calibrationSamples_==0)calibrationQuaternion_=rotation.normalized();
          else calibrationQuaternion_=QQuaternion::slerp(calibrationQuaternion_,rotation.normalized(),
                                                        1.f/float(calibrationSamples_+1));
          }
        }
        ++calibrationSamples_;
        if(calibrationSamples_==1&&guidePhase_==GuidePhase::Capture)
          setGuideText("CAPTURING: "+guidePose(calibrationStep_),"Pose detected · keep holding","#138144");
        const int required=calibrationStep_==3?50:15;
        calibrationStatus_->setText(QString("Hold: %1 · %2/%3 valid frames")
            .arg(calibrationList_->item(calibrationStep_)->text())
            .arg(calibrationSamples_).arg(required));
        if(guidePhase_==GuidePhase::Capture)
          guideDetail_->setText(QString("CAPTURING · %1/%2 valid frames")
              .arg(calibrationSamples_).arg(required));
        if(calibrationSamples_>=required) {
          if(calibrationStep_==3) {
          for(auto it=calibrationSums_.begin();it!=calibrationSums_.end();++it)
            it.value()/=double(calibrationSamples_);
          calibrated_=true;calibrating_=false;
          }
          calibrating_=false;
          calibrationList_->item(calibrationStep_)->setForeground(QColor(37,156,87));
          QApplication::beep();QTimer::singleShot(220,this,[]{QApplication::beep();});
          if(guideMode_!=0&&guidePhase_==GuidePhase::Capture) {
            const int last=guideMode_==1?2:8;
            if(calibrationStep_>=last) {
              guidePhase_=GuidePhase::Complete;
              if(guideMode_==1)bodyCalibrated_=true;else faceCalibrated_=true;
              setGuideText(guideMode_==1?"BODY CALIBRATION COMPLETE":"FACE CALIBRATION COMPLETE",
                  "All requested poses passed. You can start recording.","#138144");
              calibrationStatus_->setText("Guided calibration complete.");
            } else {
              calibrationStatus_->setText("Pose passed. Prepare for the next instruction.");
              beginGuideStep(calibrationStep_+1);
            }
          } else {
            calibrationStatus_->setText(QString("✓ %1 succeeded. Select the next pose and press Capture.")
                .arg(calibrationList_->item(calibrationStep_)->text()));
            if(calibrationStep_+1<calibrationList_->count())
              calibrationList_->setCurrentRow(calibrationStep_+1);
          }
        }
      } else if(calibrating_ && ++calibrationMisses_>=30) {
        calibrationStatus_->setText(QString("Pose not detected yet: %1. Adjust position and keep holding; %2 valid frames captured.")
            .arg(calibrationList_->item(calibrationStep_)->text()).arg(calibrationSamples_));
        calibrationMisses_=0;
        if(guidePhase_==GuidePhase::Capture) {
          setGuideText("ADJUST POSE: "+guidePose(calibrationStep_),
              QString("Not detected yet · %1 valid frames captured").arg(calibrationSamples_),"#9c6800");
        }
        if(!calibrationWarned_) {
          calibrationWarned_=true;
          QApplication::beep();
          QTimer::singleShot(220,this,[]{QApplication::beep();});
          QTimer::singleShot(440,this,[]{QApplication::beep();});
        }
      }
      if(calibrated_&&face.value("tracked").toBool()) {
        QJsonObject shapes=face.value("blendshapes").toObject();
        for(auto it=shapes.begin();it!=shapes.end();++it) {
          const double baseline=calibrationSums_.value(it.key());
          const double value=it.value().toDouble();
          shapes[it.key()]=std::clamp((value-baseline)/std::max(.05,1.-baseline),0.,1.);
        }
        face["blendshapes"]=shapes;
        QJsonObject head=face.value("head").toObject();
        const QJsonArray q=head.value("q").toArray();
        if(q.size()==4) {
          const QQuaternion measured(q[3].toDouble(),q[0].toDouble(),q[1].toDouble(),q[2].toDouble());
          const QQuaternion relative=calibrationQuaternion_.conjugated()*measured;
          head["q"]=QJsonArray{relative.x(),relative.y(),relative.z(),relative.scalar()};
          face["head"]=head;
        }
        face["neutralCalibrated"]=true;root["face"]=face;
      }
      root["quality"]=updateCaptureQuality(root);
      updateFitCheck(root);
      const QByteArray pose=QJsonDocument(root).toJson(QJsonDocument::Compact);
      if(face.value("tracked").toBool()) {
        const QJsonObject shapes=face.value("blendshapes").toObject();
        auto value=[&](const char* name){return shapes.value(QLatin1String(name)).toDouble();};
        jawMeter_->setValue(int(std::lround(value("jawOpen")*100)));
        smileMeter_->setValue(int(std::lround((value("mouthSmileLeft")+
            value("mouthSmileRight"))*50)));
        blinkMeter_->setValue(int(std::lround((value("eyeBlinkLeft")+
            value("eyeBlinkRight"))*50)));
        faceStatus_->setText(QString("Face: %1% · 51 expressions")
            .arg(int(std::lround(face.value("confidence").toDouble()*100))));
      } else {
        faceStatus_->setText("Face: searching");
        jawMeter_->setValue(0);smileMeter_->setValue(0);blinkMeter_->setValue(0);
      }
      const QJsonArray bodies=root.value("bodies").toArray();
      if(!bodies.isEmpty()) {
        const QJsonObject joints=bodies.first().toObject().value("joints").toObject();
        bool complete=true;
        for(const char* joint:{"shoulderL","shoulderR","elbowL","elbowR",
                                "wristL","wristR","hipL","hipR","kneeL","kneeR",
                                "ankleL","ankleR"})complete&=joints.contains(QLatin1String(joint));
        if(complete)++fullBodyFrames_;
      }
      maxPoseBytes_=std::max(maxPoseBytes_,int(pose.size()));
      updateSensorRow("local","USB",root.value("observed").toInt(),
                      tracked?"Tracking":"Searching");
      lan_.broadcastPose(pose);
      const HumanoidPose rig=rig_.retarget(pose,mirrorEnabled_->isChecked());
      history_.push_back(pose);historyBytes_+=size_t(pose.size());
      constexpr size_t previewLimit=64u*1024u*1024u;
      while(history_.size()>1&&(history_.size()>1800||historyBytes_>previewLimit||
          root.value("t").toDouble()-QJsonDocument::fromJson(history_.front()).object()
              .value("t").toDouble()>60.)) {
        historyBytes_-=size_t(history_.front().size());history_.pop_front();
        if(!live_&&timeline_->value()>0)timeline_->setValue(timeline_->value()-1);
      }
      timeline_->setRange(0,int(history_.size())-1);
      if(live_) showTimeline(int(history_.size())-1);
      const QString capturePath=qEnvironmentVariable("REMOLD_SOLVE_CAPTURE");
      if(!capturePath.isEmpty()&&history_.size()%10==0)
        stage_->grab().save(QString("%1.frame%2.png").arg(capturePath)
            .arg(history_.size(),4,10,QChar('0')),"PNG");
      QByteArray output=unityEnabled_->isChecked()?
          QJsonDocument(MocapRig::unityJson(pose,rig,mirrorEnabled_->isChecked())).toJson(QJsonDocument::Compact):pose;
      QByteArray recordedOutput=output;
      if(!qEnvironmentVariableIsEmpty("REMOLD_SOLVE_CAPTURE")) {
        QJsonObject diagnosticRoot=root;
        diagnosticRoot["rawBodies"]=rawRoot.value("bodies");
        diagnosticRoot["rig"]=MocapRig::unityJson(pose,rig,mirrorEnabled_->isChecked());
        recordedOutput=QJsonDocument(diagnosticRoot).toJson(QJsonDocument::Compact);
      }
      if(!network_.send(pose,&rig,output)) status_->setText(network_.error());
      else status_->setText(QString("%1 · %2 observed joints · JSON %3 · OSC %4 · VMC %5 · Face %6")
          .arg(tracked?"Tracked":"Searching")
          .arg(root.value("observed").toInt())
          .arg(network_.jsonPackets()).arg(network_.oscPackets()).arg(network_.vmcPackets())
          .arg(network_.facePackets()));
      web_.broadcast(output);
      bvh_.append(rig);
      if(recording_) {
        QByteArray packed;
        if(compactRecording_) {
          const QByteArray compressed=qCompress(recordedOutput,3);
          const quint32 size=quint32(compressed.size());
          packed.resize(4);for(int i=0;i<4;++i)packed[i]=char((size>>(i*8))&255);
          packed+=compressed;
        } else {packed=recordedOutput;packed+='\n';}
        if(file_.write(packed)!=packed.size()) {
          status_->setText(file_.errorString());stopRecording();
        } else {
          ++recorded_;recordedBytes_+=quint64(packed.size());
          if(recorded_%120==0) {
            const QStorageInfo disk(QFileInfo(file_.fileName()).absolutePath());
            storage_->setText(QString("Recorded %1 MiB · disk free %2 GiB · preview %3 MiB")
                .arg(recordedBytes_/(1024.0*1024.0),0,'f',1)
                .arg(disk.bytesAvailable()/(1024.0*1024.0*1024.0),0,'f',1)
                .arg(historyBytes_/(1024.0*1024.0),0,'f',1));
            if(disk.isValid()&&disk.bytesAvailable()<128ll*1024*1024) {
              stopRecording();storage_->setText("Recording stopped: less than 128 MiB free");
            }
          }
        }
      }
    });
    connect(worker_,&StreamWorker::message,status_,&QLabel::setText);
    connect(worker_,&StreamWorker::providerReady,this,[this](const QString& body,const QString& hands,
        const QString& face){
      providerSummary_=QString("Body %1 · hands %2 · face %3").arg(body,hands,face);
      performance_->setText(providerSummary_+(profileEnabled_->isChecked()?" · measuring…":" · timing off"));
    });
    connect(worker_,&StreamWorker::profileReady,this,[this](double body,double hands,
        double face,double lift,int frames){
      performance_->setText(providerSummary_+QString(" · %1 frames · body %2 ms · hands %3 ms · face %4 ms · depth %5 ms · filter/fit %6 ms")
          .arg(frames).arg(body,0,'f',1).arg(hands,0,'f',1).arg(face,0,'f',1)
          .arg(lift,0,'f',1).arg(lastFitMs_,0,'f',2));
    });
    worker_->startLatest();
  }
  void stop() {
    ++calibrationGeneration_;calibrating_=false;
    cancelGuide(false);
    checkActive_=false;checkPending_=false;
    checkTimer_.stop();
    if(worker_) {worker_->requestInterruption();worker_->wait();delete worker_;worker_=nullptr;}
    web_.stop();
    stopRecording();
    if(bvh_.recording()) {
      const bool saved=bvh_.stop();
      bvhButton_->setText("Record BVH");
      QApplication::beep();QTimer::singleShot(220,this,[]{QApplication::beep();});
      if(!saved) status_->setText(bvh_.error());
    }
  }
  void setGuideText(const QString& headline,const QString& detail,const QString& color="#17334b") {
    guideBanner_->setText(headline);guideDetail_->setText(detail);
    guideBanner_->setStyleSheet(QString("background:%1;color:white;font-size:36px;font-weight:bold;border-radius:8px;").arg(color));
    guideBanner_->show();guideDetail_->show();
  }
  static QString guidePose(int step) {
    static const char* names[]{"STAND FULLY IN VIEW, ARMS RELAXED",
        "HOLD A T POSE · ARMS STRAIGHT OUT","LOWER ARMS TO YOUR SIDES",
        "FACE FORWARD · RELAX YOUR FACE","HOLD A BROAD SMILE",
        "OPEN YOUR MOUTH","CLOSE BOTH EYES","TURN YOUR HEAD LEFT",
        "TURN YOUR HEAD RIGHT"};
    return step>=0&&step<9?QString::fromUtf8(names[step]):QString();
  }
  void cancelGuide(bool announce=true) {
    guideTimer_.stop();guideMode_=0;guidePhase_=GuidePhase::Idle;
    guidePositionHits_=0;calibrating_=false;++calibrationGeneration_;
    checkActive_=false;checkPending_=false;checkTimer_.stop();
    guideBanner_->hide();guideDetail_->hide();
    if(announce)calibrationStatus_->setText("Guide cancelled. Choose Body or Face to start again.");
  }
  void startGuide(bool faceMode) {
    checkActive_=false;checkPending_=false;
    checkTimer_.stop();
    if(!worker_||!worker_->isRunning()||(faceMode&&!activeFace_)||
       (!faceMode&&!activeBody_)) {
      bodyEnabled_->setChecked(!faceMode);faceEnabled_->setChecked(faceMode);
      start();
    }
    if(!worker_||!worker_->isRunning()) {
      setGuideText("SENSOR NOT READY","Start the sensor, then try calibration.","#a5232e");return;
    }
    cancelGuide(false);
    guideMode_=faceMode?2:1;guidePhase_=GuidePhase::Position;
    sideTabs_->setCurrentIndex(0);
    calibrationStep_=faceMode?3:0;calibrationList_->setCurrentRow(calibrationStep_);
    setGuideText(faceMode?"FACE POSITION CHECK":"FULL BODY POSITION CHECK",
        faceMode?"Face the sensor and follow the distance directions.":
                 "Show your head, hands, knees and feet in the video view.");
  }
  void startFitCheck() {
    if(!worker_||!worker_->isRunning())start();
    if(!worker_||!worker_->isRunning()) {
      setGuideText("SENSOR NOT READY","Choose a sensor and start tracking first.","#a5232e");return;
    }
    cancelGuide(false);checkActive_=false;checkPending_=true;
    sideTabs_->setCurrentIndex(0);
    checkCountdown_=8;
    setGuideText("FIT CHECK STARTS IN 8 SECONDS",
        activeBody_?"Move fully into view. Show head, hands and feet.":
                    "Face the camera with your face fully visible.");
    checkTimer_.start();
  }
  void updateFitCheck(const QJsonObject& root) {
    if(!checkActive_)return;
    ++checkFrames_;
    if(root.value("quality").toObject().value("state")=="green"&&activeBody_)++checkBodyGood_;
    if(root.value("face").toObject().value("tracked").toBool()&&
       root.value("face").toObject().value("confidence").toDouble()>=.8)++checkFaceGood_;
    if(checkFrames_<60) {
      guideDetail_->setText(QString("Hold your position · %1/60 frames").arg(checkFrames_));return;
    }
    checkActive_=false;
    QApplication::beep();QTimer::singleShot(220,this,[]{QApplication::beep();});
    if(activeBody_&&checkBodyGood_<30) {
      setGuideText("BODY FIT NEEDS WORK",
          "Use the video and quality marker, then run Guide body calibration.","#9c6800");
    } else if(activeFace_&&checkFaceGood_<36) {
      setGuideText("FACE FIT NEEDS WORK",
          "Center your face and adjust distance, then run Guide face calibration.","#9c6800");
    } else if(activeBody_&&!bodyCalibrated_) {
      setGuideText("FIT GOOD · CALIBRATE BODY",
          "Choose Guide body calibration before recording.","#9c6800");
    } else if(activeFace_&&!faceCalibrated_) {
      setGuideText("FIT GOOD · CALIBRATE FACE",
          "Choose Guide face calibration before recording.","#9c6800");
    } else {
      setGuideText("READY TO RECORD","Fit and guided calibration checks passed.","#138144");
    }
  }
  void beginGuideStep(int step) {
    calibrationStep_=step;calibrationSamples_=0;calibrationMisses_=0;
    calibrationWarned_=false;calibrating_=false;
    if(step==3) {calibrated_=false;calibrationSums_.clear();calibrationQuaternion_=QQuaternion();}
    calibrationList_->setCurrentRow(step);
    guidePhase_=GuidePhase::Prepare;guideCountdown_=5;
    setGuideText("NEXT: "+guidePose(step),"Get into the pose · capture starts in 5 seconds");
    guideTimer_.start();
  }
  QString positionInstruction(const QJsonObject& root) const {
    if(guideMode_==2) {
      const QJsonObject face=root.value("face").toObject();
      if(!face.value("tracked").toBool()||face.value("confidence").toDouble()<.75)
        return "FACE THE CAMERA · MOVE INTO VIEW";
      const QJsonArray points=face.value("landmarks").toArray();
      double left=1.,right=0.,top=1.,bottom=0.;int count=0;
      for(const QJsonValue& value:points) {
        const QJsonArray p=value.toArray();if(p.size()<2)continue;
        const double x=p[0].toDouble(),y=p[1].toDouble();
        if(x<0||x>1||y<0||y>1)continue;
        left=std::min(left,x);right=std::max(right,x);
        top=std::min(top,y);bottom=std::max(bottom,y);++count;
      }
      if(count<400)return "FACE THE CAMERA · MOVE INTO VIEW";
      if(bottom-top<.16)return "MOVE CLOSER TO THE CAMERA";
      if(bottom-top>.68||left<.04||right>.96||top<.04||bottom>.96)
        return "MOVE FARTHER FROM THE CAMERA";
      if((left+right)*.5<.38||(left+right)*.5>.62)
        return "CENTER YOUR FACE IN THE VIDEO";
      return {};
    }
    const QJsonArray bodies=root.value("bodies").toArray();
    if(bodies.isEmpty()) {
      const QJsonArray points=root.value("imageBody").toArray();
      if(points.size()==17) {
        const QJsonArray nose=points[0].toArray(),left=points[15].toArray(),
                         right=points[16].toArray();
        if(nose.size()==3&&nose[2].toDouble()>.3) {
          if(nose[1].toDouble()<.07||left.size()!=3||right.size()!=3||
             left[2].toDouble()<.2||right[2].toDouble()<.2)
            return "MOVE FARTHER BACK · SHOW HEAD AND FEET";
          const double height=std::max(left[1].toDouble(),right[1].toDouble())-
                              nose[1].toDouble();
          if(height<.52)return "MOVE CLOSER TO THE CAMERA";
        }
      }
      return "STEP INTO VIEW · SHOW HEAD TO FEET";
    }
    const QJsonObject joints=bodies.first().toObject().value("joints").toObject();
    auto uv=[&](const char* name){return joints.value(QLatin1String(name)).toObject().value("uv").toArray();};
    const QJsonArray head=uv("head"),leftAnkle=uv("ankleL"),rightAnkle=uv("ankleR");
    if(head.size()!=2||leftAnkle.size()!=2||rightAnkle.size()!=2)
      return "MOVE FARTHER BACK · SHOW HEAD AND FEET";
    const double top=head[1].toDouble();
    const double bottom=std::max(leftAnkle[1].toDouble(),rightAnkle[1].toDouble());
    const double height=bottom-top;
    if(top<.05||bottom>.95||height>.88)return "MOVE FARTHER FROM THE CAMERA";
    if(height<.52)return "MOVE CLOSER TO THE CAMERA";
    if(!calibrationMatches(root,0))return "SHOW BOTH ARMS AND FEET CLEARLY";
    return {};
  }
  void updateGuidePosition(const QJsonObject& root) {
    if(guidePhase_!=GuidePhase::Position)return;
    const QString instruction=positionInstruction(root);
    if(!instruction.isEmpty()) {
      guidePositionHits_=std::max(0,guidePositionHits_-2);
      setGuideText(instruction,"Follow the large instruction until the guide starts automatically.","#9c6800");
      return;
    }
    if(++guidePositionHits_>=8) {beginGuideStep(guideMode_==2?3:0);return;}
    setGuideText("GOOD DISTANCE · HOLD POSITION",
        QString("Checking framing: %1/8 stable frames").arg(guidePositionHits_),"#138144");
  }
  bool calibrationMatches(const QJsonObject& root,int step) const {
    if(step<0)return false;
    if(step<=2) {
      const QJsonArray bodies=root.value("bodies").toArray();
      if(bodies.isEmpty())return false;
      const QJsonObject joints=bodies.first().toObject().value("joints").toObject();
      auto uv=[&](const char* name)->QJsonArray {
        const QJsonObject joint=joints.value(QLatin1String(name)).toObject();
        if(joint.value("c").toDouble()<.2)return {};
        return joint.value("uv").toArray();
      };
      const auto sl=uv("shoulderL"),sr=uv("shoulderR"),el=uv("elbowL"),
          er=uv("elbowR"),wl=uv("wristL"),wr=uv("wristR"),
          hl=uv("hipL"),hr=uv("hipR"),kl=uv("kneeL"),kr=uv("kneeR"),
          al=uv("ankleL"),ar=uv("ankleR");
      if(sl.size()!=2||sr.size()!=2||el.size()!=2||er.size()!=2||
         wl.size()!=2||wr.size()!=2||hl.size()!=2||hr.size()!=2)return false;
      if(step==0)return kl.size()==2&&kr.size()==2&&al.size()==2&&ar.size()==2;
      const double shoulderY=(sl[1].toDouble()+sr[1].toDouble())*.5;
      const double hipY=(hl[1].toDouble()+hr[1].toDouble())*.5;
      if(step==1) {
        const double leftX=std::min(sl[0].toDouble(),sr[0].toDouble());
        const double rightX=std::max(sl[0].toDouble(),sr[0].toDouble());
        return std::abs(wl[1].toDouble()-shoulderY)<.14&&
            std::abs(wr[1].toDouble()-shoulderY)<.14&&
            std::abs(el[1].toDouble()-shoulderY)<.14&&
            std::abs(er[1].toDouble()-shoulderY)<.14&&
            std::min(wl[0].toDouble(),wr[0].toDouble())<leftX-.08&&
            std::max(wl[0].toDouble(),wr[0].toDouble())>rightX+.08;
      }
      return wl[1].toDouble()>shoulderY+.12&&wr[1].toDouble()>shoulderY+.12&&
          std::abs(wl[1].toDouble()-hipY)<.25&&std::abs(wr[1].toDouble()-hipY)<.25;
    }
    const QJsonObject face=root.value("face").toObject();
    if(!face.value("tracked").toBool()||face.value("confidence").toDouble()<.8)return false;
    const QJsonObject shapes=face.value("blendshapes").toObject();
    auto score=[&](const char* key){return shapes.value(QLatin1String(key)).toDouble();};
    if(step==3)return score("jawOpen")<.2&&
        (score("mouthSmileLeft")+score("mouthSmileRight"))<.5&&
        (score("eyeBlinkLeft")+score("eyeBlinkRight"))<.7;
    if(step==4)return (score("mouthSmileLeft")+score("mouthSmileRight"))>.8;
    if(step==5)return score("jawOpen")>.12;
    if(step==6)return score("eyeBlinkLeft")>.4&&score("eyeBlinkRight")>.4;
    const QJsonArray q=face.value("head").toObject().value("q").toArray();
    if(q.size()!=4)return false;
    const QQuaternion rotation(q[3].toDouble(),q[0].toDouble(),q[1].toDouble(),q[2].toDouble());
    const QVector3D forward=rotation.rotatedVector(QVector3D(0,0,1));
    const double yaw=std::atan2(forward.x(),forward.z());
    // Head yaw follows the subject's direction, while the preview is mirrored.
    return step==7?yaw>.35:yaw<-.35;
  }
  QJsonObject updateCaptureQuality(const QJsonObject& root) {
    if(!activeBody_) {
      const QJsonObject face=root.value("face").toObject();
      const bool found=face.value("tracked").toBool();
      const QJsonObject faceQuality=face.value("quality").toObject();
      const bool marginal=found&&faceQuality.value("state")=="yellow";
      const QString color=!found?"#a5232e":marginal?"#aa7900":"#138144";
      quality_->setText(!found?"FACE LOST":marginal?"FACE MARGINAL":"FACE GOOD");
      const QString reason=faceQuality.value("reason").toString();
      qualityDetail_->setText(reason=="turn-toward-camera"?
          "Turn more toward the camera for reliable expressions":
          reason=="adjust-face-distance"?
          "Face fills tracking crop; step back if close and hold steady":
          reason=="reduce-head-tilt"?"Reduce head tilt for reliable expressions":
          !found?"Move your face into the video view":
          marginal?"Face landmarks are uncertain; hold steady":"Face visible · body tracking is off");
      quality_->setStyleSheet(QString("background:%1;color:white;font-size:23px;font-weight:bold;border-radius:8px;").arg(color));
      return {{"state",!found?"red":marginal?"yellow":"green"},{"source","face"},
              {"reason",found?reason:"face-lost"}};
    }
    const QJsonArray bodies=root.value("bodies").toArray();
    if(bodies.isEmpty()) {
      quality_->setText("UNUSABLE");qualityDetail_->setText("No body solve. Move fully into view.");
      quality_->setStyleSheet("background:#a5232e;color:white;font-size:23px;font-weight:bold;border-radius:8px;");
      return {{"state","red"},{"reason","no-body"}};
    }
    const QJsonObject joints=bodies.first().toObject().value("joints").toObject();
    const char* needed[]{"shoulderL","shoulderR","elbowL","elbowR","wristL","wristR",
                          "hipL","hipR","kneeL","kneeR","ankleL","ankleR"};
    int missing=0;double weakest=1.,margin=1.;QString firstMissing;
    for(const char* name:needed) {
      const QJsonObject joint=joints.value(QLatin1String(name)).toObject();
      const QJsonArray uv=joint.value("uv").toArray();
      if(uv.size()!=2||joint.value("c").toDouble()<.15) {
        ++missing;if(firstMissing.isEmpty())firstMissing=QLatin1String(name);
        continue;
      }
      weakest=std::min(weakest,joint.value("c").toDouble());
      margin=std::min({margin,uv[0].toDouble(),1.-uv[0].toDouble(),
                      uv[1].toDouble(),1.-uv[1].toDouble()});
    }
    int badLinks=0;
    auto distance=[&](const char* parent,const char* child) {
      const QJsonArray a=joints.value(QLatin1String(parent)).toObject().value("p").toArray();
      const QJsonArray b=joints.value(QLatin1String(child)).toObject().value("p").toArray();
      if(a.size()!=3||b.size()!=3)return 0.;
      return double(QVector3D(float(a[0].toDouble()-b[0].toDouble()),
                       float(a[1].toDouble()-b[1].toDouble()),
                       float(a[2].toDouble()-b[2].toDouble())).length());
    };
    for(const auto& pair:std::array<std::pair<const char*,const char*>,8>{{
            {"shoulderL","elbowL"},{"shoulderR","elbowR"},
            {"elbowL","wristL"},{"elbowR","wristR"},
            {"hipL","kneeL"},{"hipR","kneeR"},
            {"kneeL","ankleL"},{"kneeR","ankleR"}}})
      if(distance(pair.first,pair.second)>(pair.first[0]=='h'||pair.first[0]=='k'?.95:.7))++badLinks;
    QString state,reason,color;
    if(missing>=3) {state="UNUSABLE";reason=QString("%1 joints missing; move fully into view").arg(missing);color="#a5232e";}
    else if(badLinks>=2) {state="UNUSABLE";reason="Implausible limb geometry; adjust position";color="#a5232e";}
    else if(missing>0) {state="MARGINAL";reason=firstMissing+" is not reliably tracked";color="#aa7900";}
    else if(badLinks>0) {state="MARGINAL";reason="A limb solve is unstable";color="#aa7900";}
    else if(margin<.08) {state="MARGINAL";reason="A limb is near the frame edge; recenter or step back";color="#aa7900";}
    else if(weakest<.18) {state="MARGINAL";reason="Weak joint confidence; improve visibility";color="#aa7900";}
    else {state="GOOD";reason="Full body in view with reliable joints";color="#138144";}
    quality_->setText(state);qualityDetail_->setText(reason);
    quality_->setStyleSheet(QString("background:%1;color:white;font-size:23px;font-weight:bold;border-radius:8px;").arg(color));
    return {{"state",state=="GOOD"?"green":state=="MARGINAL"?"yellow":"red"},
            {"reason",reason},{"missingJoints",missing},{"badLinks",badLinks},
            {"weakestConfidence",weakest},
            {"frameMargin",margin}};
  }
  void showTimeline(int index) {
    if(clipFile_.isOpen()&&clipIndex_) {
      const QJsonObject frame=clipIndex_->frameAt(clipFile_,index);
      if(frame.isEmpty())return;
      timeline_->setValue(index);stage_->setFrame({});
      stage_->setPose(QJsonDocument(frame).toJson(QJsonDocument::Compact));
      const QJsonObject face=frame.value("face").toObject();
      const QJsonObject shapes=face.value("blendshapes").toObject();
      auto amount=[&](const char* name){return shapes.value(QLatin1String(name)).toDouble();};
      jawMeter_->setValue(int(std::lround(amount("jawOpen")*100)));
      smileMeter_->setValue(int(std::lround((amount("mouthSmileLeft")+
          amount("mouthSmileRight"))*50)));
      blinkMeter_->setValue(int(std::lround((amount("eyeBlinkLeft")+
          amount("eyeBlinkRight"))*50)));
      faceStatus_->setText(!face.value("tracked").toBool()?"Face: none in clip":
          face.value("registration").toObject().value("state").toString().startsWith("aligned")?
          "Face: aligned mesh + expressions":"Face: expressions only");
      timelineLabel_->setText(QString("%1 / %2 s")
          .arg(frame.value("t").toDouble()-clipIndex_->firstTime,0,'f',1)
          .arg(clipIndex_->lastTime-clipIndex_->firstTime,0,'f',1));
      return;
    }
    if(index<0||index>=int(history_.size()))return;
    timeline_->setValue(index);
    if(selectedSensor_=="local") {stage_->setFrame(lastFrame_);stage_->setPose(history_[size_t(index)]);}
    const QJsonObject root=QJsonDocument::fromJson(history_[size_t(index)]).object();
    const double newest=QJsonDocument::fromJson(history_.back()).object().value("t").toDouble();
    const double age=std::max(0.,newest-root.value("t").toDouble());
    timelineLabel_->setText(live_?"LIVE":QString("-%1 s").arg(age,0,'f',1));
  }
  void openClip() {
    if(indexThread_||editorThread_)return;
    const QString path=QFileDialog::getOpenFileName(this,"Open motion clip",{},
        "Motion clips (*.rmclip *.jsonl)");
    if(path.isEmpty())return;
    stop();clipFile_.close();clipIndex_.reset();history_.clear();historyBytes_=0;
    openClipButton_->setEnabled(false);status_->setText("Indexing clip…");
    auto index=std::make_shared<MocapClipIndex>();
    indexThread_=QThread::create([path,index]{*index=MocapClipIndex::build(path);});
    connect(indexThread_,&QThread::finished,this,[this,index]{
      indexThread_=nullptr;openClipButton_->setEnabled(true);
      if(!index->error.isEmpty()){status_->setText("Clip open failed: "+index->error);return;}
      clipFile_.setFileName(index->source);
      if(!clipFile_.open(QIODevice::ReadOnly)){status_->setText(clipFile_.errorString());return;}
      clipIndex_=index;live_=false;playing_=false;playButton_->setText("Play");
      timeline_->setRange(0,index->frames-1);viewMode_->setCurrentIndex(2);
      showTimeline(0);
      status_->setText(QString("Opened %1 frames · sparse on-disk index").arg(index->frames));
    });
    connect(indexThread_,&QThread::finished,indexThread_,&QObject::deleteLater);
    indexThread_->start();
  }
  void toggleBvh() {
    if(bvh_.recording()) {
      const bool saved=bvh_.stop();bvhButton_->setText("Record BVH");
      QApplication::beep();QTimer::singleShot(220,this,[]{QApplication::beep();});
      status_->setText(saved?"BVH recording saved":bvh_.error());return;
    }
    const QString folder=QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString suggested=folder+"/Kinect-mocap-"+
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")+".bvh";
    const QString path=QFileDialog::getSaveFileName(this,"Record humanoid animation",suggested,
                                                    "Biovision Hierarchy (*.bvh)");
    if(path.isEmpty()) return;
    if(!bvh_.start(path)) {status_->setText(bvh_.error());return;}
    QApplication::beep();
    bvhButton_->setText("Stop BVH");
  }
  void editRecording() {
    if(editorThread_||indexThread_)return;
    QDialog dialog(this);dialog.setWindowTitle("Process motion capture");
    auto* layout=new QVBoxLayout(&dialog);auto* form=new QFormLayout;
    auto pathRow=[&](const QString& name,bool save,QLineEdit*& edit) {
      auto* holder=new QWidget(&dialog);auto* row=new QHBoxLayout(holder);
      row->setContentsMargins(0,0,0,0);edit=new QLineEdit(holder);
      auto* browse=new QPushButton("Browse…",holder);row->addWidget(edit,1);row->addWidget(browse);
      connect(browse,&QPushButton::clicked,&dialog,[dialogPtr=&dialog,edit,name,save]{
        const QString path=save?QFileDialog::getSaveFileName(dialogPtr,name,{},
            "Remold compact clip (*.rmclip);;JSON Lines (*.jsonl)"):
            QFileDialog::getOpenFileName(dialogPtr,name,{},"Motion clips (*.jsonl *.rmclip)");
        if(!path.isEmpty())edit->setText(path);
      });
      form->addRow(name,holder);
    };
    QLineEdit *source,*face,*output;
    pathRow("Body or combined recording",false,source);
    pathRow("Optional face recording",false,face);
    pathRow("Processed output",true,output);
    connect(source,&QLineEdit::textChanged,&dialog,[output](const QString& path){
      if(output->text().isEmpty()&&!path.isEmpty()) {
        const QFileInfo info(path);
        output->setText(info.dir().filePath(info.completeBaseName()+"-edited.rmclip"));
      }
    });
    auto seconds=[&](const QString& label,double value) {
      auto* spin=new QDoubleSpinBox(&dialog);spin->setRange(0,86400);spin->setDecimals(2);
      spin->setSuffix(" s");spin->setValue(value);form->addRow(label,spin);return spin;
    };
    auto* trimStart=seconds("Trim from start",0);
    auto* trimEnd=seconds("Trim end time (0 = full)",0);
    auto* faceOffset=seconds("Face sync offset",0);
    faceOffset->setRange(-86400,86400);
    auto* loopBlend=seconds("Loop crossfade (0 = off)",0);
    auto* smoothing=new QSlider(Qt::Horizontal,&dialog);
    smoothing->setRange(0,100);smoothing->setValue(0);
    form->addRow("Noise reduction",smoothing);
    auto* inPlace=new QCheckBox("Remove root travel for an in-place loop",&dialog);
    inPlace->setChecked(true);form->addRow(inPlace);
    auto* exportBvh=new QCheckBox("Export edited BVH alongside JSONL",&dialog);
    form->addRow(exportBvh);
    layout->addLayout(form);
    auto* hint=new QLabel("Processing writes a new clip with bounded memory. Looping selects a matching pose seam, blends the overlap, removes root travel when requested, and holds detected planted feet. Separate face mesh alignment needs a visible nose and both ears in the body clip.",&dialog);
    hint->setWordWrap(true);layout->addWidget(hint);
    auto* buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel,&dialog);
    connect(buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept);
    connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);layout->addWidget(buttons);
    if(dialog.exec()!=QDialog::Accepted)return;
    MocapClipOptions options;
    options.source=source->text();options.face=face->text();options.output=output->text();
    options.trimStart=trimStart->value();options.trimEnd=trimEnd->value();
    options.faceOffset=faceOffset->value();options.loopBlend=loopBlend->value();
    options.smoothing=smoothing->value();options.inPlaceLoop=inPlace->isChecked();
    if(exportBvh->isChecked()) {
      options.bvhOutput=options.output;
      if(options.bvhOutput.endsWith(".jsonl",Qt::CaseInsensitive))options.bvhOutput.chop(6);
      else if(options.bvhOutput.endsWith(".rmclip",Qt::CaseInsensitive))options.bvhOutput.chop(7);
      options.bvhOutput+=".bvh";
    }
    auto result=std::make_shared<MocapClipResult>();
    editButton_->setEnabled(false);status_->setText("Processing clip…");
    editorThread_=QThread::create([options,result]{*result=MocapClipProcessor::process(options);});
    connect(editorThread_,&QThread::finished,this,[this,result]{
      status_->setText(!result->error.isEmpty()?"Clip processing failed: "+result->error:
          !result->warning.isEmpty()?QString("Processed %1 frames · %2")
              .arg(result->frames).arg(result->warning):
          result->seamEndFrame>0?QString("Processed %1 frames · seam %2–%3 · score %4 · %5 planted foot frames")
              .arg(result->frames).arg(result->seamStartFrame).arg(result->seamEndFrame)
              .arg(result->seamScore,0,'f',3).arg(result->plantedFootFrames):
          QString("Processed %1 frames").arg(result->frames));
      editButton_->setEnabled(true);editorThread_=nullptr;
    });
    connect(editorThread_,&QThread::finished,editorThread_,&QObject::deleteLater);
    editorThread_->start();
  }
  void toggleRecording() {
    if(recording_) {stopRecording();return;}
    const QString folder=QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    compactRecording_=recordFormat_->currentIndex()==0;
    const QString suggested=folder+"/Kinect-mocap-"+
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")+
        (compactRecording_?".rmclip":".jsonl");
    const QString path=QFileDialog::getSaveFileName(this,"Record camera-space poses",suggested,
        compactRecording_?"Remold compact clip (*.rmclip)":"JSON Lines (*.jsonl)");
    if(path.isEmpty()) return;
    file_.setFileName(path);
    if(!file_.open(QIODevice::WriteOnly)) {status_->setText(file_.errorString());return;}
    if(compactRecording_&&file_.write("RMC1",4)!=4) {
      status_->setText(file_.errorString());file_.cancelWriting();return;
    }
    recorded_=0;recordedBytes_=compactRecording_?4:0;recording_=true;
    recordButton_->setText("Stop recording");
    QApplication::beep();
  }
  void stopRecording() {
    if(!recording_) return;
    const bool saved=file_.commit();recording_=false;
    recordButton_->setText("Record clip");
    QApplication::beep();QTimer::singleShot(220,this,[]{QApplication::beep();});
    status_->setText(saved?QString("Recorded %1 pose frames").arg(recorded_):file_.errorString());
    if(saved)storage_->setText(QString("Saved %1 MiB · preview remains capped at 64 MiB")
        .arg(recordedBytes_/(1024.0*1024.0),0,'f',1));
  }
  std::vector<Device> found_;
  QComboBox* devices_=nullptr;QCheckBox* jsonEnabled_=nullptr;QCheckBox* oscEnabled_=nullptr;
  QCheckBox* webEnabled_=nullptr;QCheckBox* vmcEnabled_=nullptr;QCheckBox* faceCapEnabled_=nullptr;
  QCheckBox* unityEnabled_=nullptr;QCheckBox* mirrorEnabled_=nullptr;
  QSpinBox* jsonPort_=nullptr;QSpinBox* oscPort_=nullptr;QSpinBox* webPort_=nullptr;QSpinBox* vmcPort_=nullptr;QSpinBox* faceCapPort_=nullptr;
  MocapStageView* stage_=nullptr;QComboBox* viewMode_=nullptr;
  QTableWidget* sensors_=nullptr;QLineEdit* lanUrl_=nullptr;
  QLineEdit* lanCert_=nullptr;QLineEdit* lanKey_=nullptr;QLineEdit* lanCa_=nullptr;
  QSpinBox* lanPort_=nullptr;QLabel* lanStatus_=nullptr;
  QLabel* faceStatus_=nullptr;QProgressBar* jawMeter_=nullptr;
  QProgressBar* smileMeter_=nullptr;QProgressBar* blinkMeter_=nullptr;
  QLabel* calibrationStatus_=nullptr;QHash<QString,double> calibrationSums_;
  QLabel* guideBanner_=nullptr;QLabel* guideDetail_=nullptr;
  QTimer guideTimer_;GuidePhase guidePhase_=GuidePhase::Idle;
  int guideMode_=0,guidePositionHits_=0,guideCountdown_=0;
  QTabWidget* sideTabs_=nullptr;
  bool bodyCalibrated_=false,faceCalibrated_=false;
  bool checkActive_=false,checkPending_=false;
  QTimer checkTimer_;int checkCountdown_=0;
  int checkFrames_=0,checkBodyGood_=0,checkFaceGood_=0;
  QLabel* quality_=nullptr;QLabel* qualityDetail_=nullptr;
  QQuaternion calibrationQuaternion_;int calibrationSamples_=0,calibrationMisses_=0;
  int calibrationGeneration_=0;bool calibrationWarned_=false;
  bool calibrating_=false,calibrated_=false;
  QListWidget* calibrationList_=nullptr;int calibrationStep_=-1;
  QCheckBox* bodyEnabled_=nullptr;QCheckBox* faceEnabled_=nullptr;
  QSlider* stability_=nullptr;
  QComboBox* bodyProvider_=nullptr;QComboBox* handProvider_=nullptr;QComboBox* faceProvider_=nullptr;
  QCheckBox* fitEnabled_=nullptr;QCheckBox* profileEnabled_=nullptr;
  QLabel* performance_=nullptr;
  QString providerSummary_;
  bool activeBody_=true,activeFace_=true;
  QSlider* timeline_=nullptr;QLabel* timelineLabel_=nullptr;
  QLabel* storage_=nullptr;quint64 recordedBytes_=0;
  QPushButton* editButton_=nullptr;QThread* editorThread_=nullptr;
  QPushButton* openClipButton_=nullptr;QThread* indexThread_=nullptr;
  QFile clipFile_;std::shared_ptr<MocapClipIndex> clipIndex_;
  QPushButton* playButton_=nullptr;QPushButton* liveButton_=nullptr;
  QLabel* status_=nullptr;QPushButton* recordButton_=nullptr;QPushButton* bvhButton_=nullptr;
  QComboBox* recordFormat_=nullptr;bool compactRecording_=false;
  StreamWorker* worker_=nullptr;MocapNetwork network_;MocapWebSocket web_;MocapRig rig_;MocapBvh bvh_;QSaveFile file_;
  MocapPoseSmoother smoother_;
  ArticulatedPoseFit bodyFit_;
  double fitMs_=0,lastFitMs_=0;int fitFrames_=0;
  LanPoseLink lan_;QHash<QString,QByteArray> remotePoses_;
  QString selectedSensor_="local";QImage lastFrame_;
  std::deque<QByteArray> history_;size_t historyBytes_=0;QTimer playback_;bool live_=true,playing_=false;
  bool recording_=false;int recorded_=0,faceFrames_=0,fullBodyFrames_=0,maxPoseBytes_=0;
};
#endif

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QString preferredId = {}) : preferredId_(std::move(preferredId)) {
    setWindowTitle("Kinect Remold"); resize(1080, 780);
    auto* central = new QWidget(this); auto* layout = new QVBoxLayout(central);
    auto* row = new QHBoxLayout;
    devices_ = new QComboBox; modes_ = new QComboBox;
    modes_->addItems({"RGB", "Depth", "Infrared", "RGB HQ (Xbox 360)"});
#ifdef REMOLD_HAS_ONNX
    modes_->addItem("Body pose");
    modes_->addItem("Body + hands");
#ifdef Q_OS_MACOS
    const QString depthModel = QCoreApplication::applicationDirPath() +
        "/../Resources/models/SynSkeleton.onnx";
#else
    const QString depthModel = QCoreApplication::applicationDirPath() +
        "/models/SynSkeleton.onnx";
#endif
    if (QFileInfo::exists(depthModel)) modes_->addItem("Depth skeleton (58 landmarks)");
#endif
#ifdef Q_OS_MACOS
    modes_->addItems({"Hand landmarks", "Face landmarks"});
#endif
    auto* refreshButton = new QPushButton("Refresh"); auto* probe = new QPushButton("Probe USB");
    auto* openWindow = new QPushButton("Open sensor window");
    auto* surveillance = new QPushButton("Surveillance");
    auto* microphones = new QPushButton("Microphones");
    auto* acoustic = new QPushButton("Acoustic Scanner");
    auto* scanner = new QPushButton("3D Scanner");
#ifdef REMOLD_HAS_ONNX
    auto* mocap = new QPushButton("Motion Capture");
#endif
    auto* calibration = new QPushButton("Sensor Calibration");
#ifndef Q_OS_MACOS
    probe->setVisible(false);
#endif
    auto* start = new QPushButton("Start"); auto* stop = new QPushButton("Stop");
    auto* snapshot = new QPushButton("Save frame");
    auto* cloud = new QPushButton("Export depth PLY");
    auto* mesh = new QPushButton("Export mesh OBJ");
#ifdef REMOLD_HAS_ONNX
    auto* poseExport = new QPushButton("Save 3D pose JSON");
#endif
    led_ = new QComboBox;
    led_->addItems({"LED off", "LED green", "LED red", "LED blink"});
    auto* setLed = new QPushButton("Set LED");
    tilt_ = new QSpinBox;
    tilt_->setRange(-27, 27); tilt_->setSuffix("°");
    auto* setTilt = new QPushButton("Set tilt");
    recordButton_ = new QPushButton("Record video");
    row->addWidget(devices_, 1); row->addWidget(modes_); row->addWidget(refreshButton); row->addWidget(openWindow); row->addWidget(probe); row->addWidget(start); row->addWidget(stop);
    layout->addLayout(row);
    auto* modules = new QHBoxLayout;
    modules->addWidget(surveillance); modules->addWidget(microphones);
    modules->addWidget(acoustic); modules->addWidget(calibration);
    modules->addWidget(scanner);
#ifdef REMOLD_HAS_ONNX
    modules->addWidget(mocap);
#endif
    modules->addStretch();
    layout->addLayout(modules);
    auto* controls = new QHBoxLayout;
    controls->addWidget(led_); controls->addWidget(setLed);
    controls->addWidget(tilt_); controls->addWidget(setTilt); controls->addStretch();
    controls->addWidget(recordButton_); controls->addWidget(snapshot);
    controls->addWidget(cloud); controls->addWidget(mesh);
#ifdef REMOLD_HAS_ONNX
    controls->addWidget(poseExport);
#endif
    layout->addLayout(controls);
    preview_ = new QLabel("Connect a Kinect, then select a stream.");
    preview_->setAlignment(Qt::AlignCenter); preview_->setMinimumSize(640, 480);
    preview_->setStyleSheet("background:#151b22;color:#e5e9ef;");
    layout->addWidget(preview_, 1);
    status_ = new QLabel("Searching for sensors…"); layout->addWidget(status_);
    setCentralWidget(central);
    connect(refreshButton, &QPushButton::clicked, this, &MainWindow::refresh);
    connect(openWindow, &QPushButton::clicked, this, [this] {
      const QString id = devices_->currentData().toString();
      if (id.isEmpty()) { status_->setText("Select a sensor first"); return; }
      auto* window = new MainWindow(id);
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
    connect(surveillance, &QPushButton::clicked, this, [] {
      auto* window = new SurveillanceWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
    connect(microphones, &QPushButton::clicked, this, [] {
      auto* window = new MicrophoneWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
    connect(acoustic, &QPushButton::clicked, this, [] {
      auto* window = new AcousticWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
    connect(calibration, &QPushButton::clicked, this, [] {
      auto* window = new CalibrationWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
    connect(scanner, &QPushButton::clicked, this, [] {
      auto* window = new ScannerWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
#ifdef REMOLD_HAS_ONNX
    connect(mocap, &QPushButton::clicked, this, [] {
      auto* window = new MotionCaptureWindow;
      window->setAttribute(Qt::WA_DeleteOnClose);
      window->show();
    });
#endif
    connect(probe, &QPushButton::clicked, this, &MainWindow::probeSelected);
    connect(start, &QPushButton::clicked, this, &MainWindow::startStream);
    connect(stop, &QPushButton::clicked, this, &MainWindow::stopStream);
    connect(snapshot, &QPushButton::clicked, this, &MainWindow::saveFrame);
    connect(cloud, &QPushButton::clicked, this, &MainWindow::exportDepthPly);
    connect(mesh, &QPushButton::clicked, this, &MainWindow::exportDepthObj);
#ifdef REMOLD_HAS_ONNX
    connect(poseExport, &QPushButton::clicked, this, &MainWindow::savePose);
#endif
    connect(setLed, &QPushButton::clicked, this, &MainWindow::setLed);
    connect(setTilt, &QPushButton::clicked, this, &MainWindow::setTilt);
    connect(devices_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this, setLed, setTilt](int index) {
      const bool xbox360 = index >= 0 && index < int(found_.size()) &&
          found_[size_t(index)].generation == QStringLiteral("xbox-360") &&
          found_[size_t(index)].state == QStringLiteral("Ready");
      led_->setEnabled(xbox360); setLed->setEnabled(xbox360);
      const bool motor = xbox360 && !found_[size_t(index)].control.isEmpty();
      tilt_->setEnabled(motor); setTilt->setEnabled(motor);
      if (!xbox360 && modes_->currentIndex() == 3) modes_->setCurrentIndex(0);
    });
    connect(devices_, QOverload<int>::of(&QComboBox::activated), this,
            [this](int) { preferredId_ = devices_->currentData().toString(); });
    connect(recordButton_, &QPushButton::clicked, this, &MainWindow::toggleRecording);
    auto* timer = new QTimer(this); connect(timer, &QTimer::timeout, this, &MainWindow::refresh); timer->start(2000);
    refresh();
  }
  ~MainWindow() override { stopStream(); }
  int checkLiveStreams() {
    refresh();
    const auto ready = std::find_if(found_.begin(), found_.end(), [](const Device& device) {
      return device.state == QStringLiteral("Ready") && !device.camera.isEmpty();
    });
    if (ready == found_.end()) return 2;
    devices_->setCurrentIndex(int(ready - found_.begin()));
    for (int mode = 0; mode <= 2; ++mode) {
      modes_->setCurrentIndex(mode);
      startStream();
      QElapsedTimer timeout; timeout.start();
      while (timeout.elapsed() < 10000 && (mode == 1 ? depth_.isEmpty() : lastImage_.isNull())) {
        QApplication::processEvents();
        if (worker_ && !worker_->isRunning()) break;
        QThread::msleep(20);
      }
      const bool valid = mode == 1 ? !depth_.isEmpty() : !lastImage_.isNull();
      stopStream();
      if (!valid) return mode == 0 ? 3 : mode == 1 ? 4 : 5;
    }
    return 0;
  }
  int checkLiveBodyHands() {
#ifndef REMOLD_HAS_ONNX
    return 2;
#else
    refresh();
    const auto ready = std::find_if(found_.begin(), found_.end(), [](const Device& device) {
      return device.state == QStringLiteral("Ready") && !device.camera.isEmpty();
    });
    if (ready == found_.end()) return 3;
    devices_->setCurrentIndex(int(ready - found_.begin()));
    modes_->setCurrentIndex(modes_->findText("Body + hands"));
    startStream();
    QElapsedTimer timeout; timeout.start();
    while (timeout.elapsed() < 15000 && (lastImage_.isNull() || poseJson_.isEmpty())) {
      QApplication::processEvents();
      if (worker_ && !worker_->isRunning()) break;
      QThread::msleep(20);
    }
    const bool success = !lastImage_.isNull() && !poseJson_.isEmpty();
    const QString capturePath=qEnvironmentVariable("REMOLD_PREVIEW_CAPTURE");
    if(success&&!capturePath.isEmpty()) lastImage_.save(capturePath,"PNG");
    if(success) fprintf(stderr,"RGBD pose: %d observed joints\n",
        QJsonDocument::fromJson(poseJson_).object().value("observed").toInt());
    stopStream();
    return success ? 0 : 4;
#endif
  }
  int checkColorCalibration() {
    refresh();
    const auto ready=std::find_if(found_.begin(),found_.end(),[](const Device& d){
      return d.state=="Ready"&&d.generation=="xbox-one"&&!d.camera.isEmpty();
    });
    if(ready==found_.end()) return 2;
    devices_->setCurrentIndex(int(ready-found_.begin()));
    modes_->setCurrentIndex(1);startStream();
    QElapsedTimer timeout;timeout.start();
    while(timeout.elapsed()<10000&&depth_.isEmpty()) {
      QApplication::processEvents();QThread::msleep(20);
    }
    stopStream();
    if(depthExtension_.size()!=sizeof(KinectOneRemoldProtocol::ReplyExtension)&&
       depthExtension_.size()!=sizeof(KinectOneRemoldProtocol::ReplyExtensionV2)) return 3;
    const char* data=depthExtension_.constData();
    auto field=[&](int offset){float value=0;std::memcpy(&value,data+offset,4);return value;};
    fprintf(stderr,"factory color flags=%u fx=%.3f fy=%.3f cx=%.3f cy=%.3f shiftD=%.3f shiftM=%.3f\n",
            u32(data+12),field(40),field(44),field(48),field(52),field(56),field(60));
#ifdef REMOLD_HAS_ONNX
    if(depthExtension_.size()>=144&&(u32(data+12)&4u)) {
      DepthIntrinsics camera{depthWidth_,depthHeight_,depthFx_,depthFy_,depthCx_,depthCy_,depthExtension_};
      float u=0,v=0;
      if(!RgbdRegistration::projectKinectOne(camera,camera.width/2,camera.height/2,
                                              2000,1920,1080,u,v)) return 4;
      fprintf(stderr,"factory depth center at 2m -> RGB %.1f, %.1f\n",u,v);
      if(u<300||u>1600||v<150||v>900)return 5;
    }
#endif
    return 0;
  }
private slots:
  void refresh() {
    if (worker_ && worker_->isRunning()) return;
    if (worker_) stopStream();
    const QString selected = devices_->currentData().toString().isEmpty() ? preferredId_ :
        devices_->currentData().toString();
    found_ = discoverAt(sdkPath(false));
    auto one = discoverAt(sdkPath(true)); found_.insert(found_.end(), one.begin(), one.end());
    devices_->clear();
    for (const auto& device : found_) devices_->addItem(device.label + " — " + device.state, device.id);
    const int index = devices_->findData(selected); if (index >= 0) devices_->setCurrentIndex(index);
    status_->setText(found_.empty() ? "No Kinect service or sensor found" : QString::number(found_.size()) + " sensor(s) found");
  }
  void startStream() {
    stopStream();
    depth_.clear(); depthExtension_.clear(); poseJson_.clear(); lastImage_ = {};
    depthCalibration_ = DepthCalibrationProfile();
    calibrationLoaded_ = false;
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size())) return;
    const Device device = found_[size_t(index)];
    if (device.camera.isEmpty()) { status_->setText("Camera is not ready: " + device.state); return; }
    const int selectedMode = modes_->currentIndex();
    if (selectedMode == 3 && device.generation != QStringLiteral("xbox-360")) {
      status_->setText("RGB HQ requires an Xbox 360 Kinect"); return;
    }
    const bool depthSkeleton = modes_->currentText().startsWith("Depth skeleton");
    const quint32 mask = selectedMode == 1 || depthSkeleton ? 4u : selectedMode == 2 ? 2u :
        selectedMode == 3 ? 8u : selectedMode >= 4 ? 5u : 1u;
    const QString mode = modes_->currentText();
    const int overlay = depthSkeleton ? 4 : mode == "Body + hands" ? 5 :
        mode == "Body pose" ? 1 : mode == "Hand landmarks" ? 2 :
        mode == "Face landmarks" ? 3 : 0;
    worker_ = new StreamWorker(device, mask, overlay, this);
    connect(worker_, &StreamWorker::frameReady, this, [this](const QImage& image) {
      lastImage_ = image;
      preview_->setPixmap(QPixmap::fromImage(image).scaled(preview_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      if (recorder_.active() && !recorder_.append(image)) {
        status_->setText(recorder_.error()); recorder_.stop(); recordButton_->setText("Record video");
      }
    });
    connect(worker_, &StreamWorker::depthReady, this,
            [this](const QByteArray& depth, int width, int height, double fx, double fy,
                   double cx, double cy, const QByteArray& extension) {
      depth_ = depth; depthWidth_ = width; depthHeight_ = height;
      depthFx_ = fx; depthFy_ = fy; depthCx_ = cx; depthCy_ = cy;
      depthExtension_ = extension;
      if (!calibrationLoaded_) {
        calibrationLoaded_ = true;
        const QString id = devices_->currentData().toString();
        depthCalibration_.load(calibrationPath(id), id, width, height);
      }
    });
    connect(worker_, &StreamWorker::poseReady, this, [this](const QByteArray& pose){
      poseJson_=pose;
    });
    connect(worker_, &StreamWorker::message, status_, &QLabel::setText);
    worker_->startLatest();
  }
  void probeSelected() {
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size())) return;
    const Device& device = found_[size_t(index)];
    status_->setText(probeAt(device.sdk, device.id));
  }
  void setLed() {
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size()) ||
        found_[size_t(index)].generation != QStringLiteral("xbox-360")) return;
    static const QByteArray actions[]{"LedOff", "LedGreen", "LedRed", "LedBlink"};
    status_->setText(controlAt(found_[size_t(index)].sdk, found_[size_t(index)].id,
                               actions[std::clamp(led_->currentIndex(), 0, 3)]));
  }
  void setTilt() {
    const int index = devices_->currentIndex();
    if (index < 0 || index >= int(found_.size()) ||
        found_[size_t(index)].generation != QStringLiteral("xbox-360")) return;
    status_->setText(tiltAt(found_[size_t(index)], tilt_->value()));
  }
  void saveFrame() {
    if (lastImage_.isNull()) { status_->setText("No frame to save"); return; }
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    const QString suggested = folder + "/Kinect-" +
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") + ".png";
    const QString path = QFileDialog::getSaveFileName(this, "Save Kinect frame", suggested, "PNG image (*.png)");
    if (path.isEmpty()) return;
    status_->setText(lastImage_.save(path, "PNG") ? "Saved " + path : "Could not save frame");
  }
  void savePose() {
    if(poseJson_.isEmpty()) {status_->setText("No synchronized 3D pose available");return;}
    const QJsonObject pose=QJsonDocument::fromJson(poseJson_).object();
    if(pose.value("observed").toInt()<1) {status_->setText("No observed 3D joints yet");return;}
    const QString folder=QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString suggested=folder+"/Kinect-pose-"+
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")+".json";
    const QString path=QFileDialog::getSaveFileName(this,"Save metric pose",suggested,
                                                    "JSON pose (*.json)");
    if(path.isEmpty()) return;
    QSaveFile file(path);
    if(!file.open(QIODevice::WriteOnly)||file.write(poseJson_)!=poseJson_.size()||!file.commit()) {
      status_->setText(file.errorString());return;
    }
    status_->setText("Saved " + path);
  }
  void exportDepthPly() {
    if (depth_.isEmpty() || depthFx_ <= 0 || depthFy_ <= 0) {
      status_->setText("No calibrated metric depth frame available"); return;
    }
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString suggested = folder + "/Kinect-depth-" +
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") + ".ply";
    const QString path = QFileDialog::getSaveFileName(this, "Export depth point cloud", suggested,
                                                     "PLY point cloud (*.ply)");
    if (path.isEmpty()) return;
    QSaveFile output(path);
    if (!output.open(QIODevice::WriteOnly)) { status_->setText(output.errorString()); return; }
    const int pixels = depthWidth_ * depthHeight_;
    int vertices = 0;
    for (int i = 0; i < pixels; ++i) if (u16(depth_.constData() + i * 2)) ++vertices;
    const QByteArray header = "ply\nformat binary_little_endian 1.0\n" 
        "element vertex " + QByteArray::number(vertices) +
        "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    if (output.write(header) != header.size()) { status_->setText(output.errorString()); return; }
    for (int y = 0; y < depthHeight_; ++y) for (int x = 0; x < depthWidth_; ++x) {
      float point[3];
      if (!depthPoint(x, y, point)) continue;
      if (output.write(reinterpret_cast<const char*>(point), sizeof(point)) != sizeof(point)) {
        status_->setText(output.errorString()); return;
      }
    }
    status_->setText(output.commit() ? "Exported " + QString::number(vertices) + " points" : output.errorString());
  }
  void exportDepthObj() {
    if (depth_.isEmpty() || depthFx_ <= 0 || depthFy_ <= 0) {
      status_->setText("No calibrated metric depth frame available"); return;
    }
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString suggested = folder + "/Kinect-mesh-" +
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") + ".obj";
    const QString path = QFileDialog::getSaveFileName(this, "Export single-frame depth mesh", suggested,
                                                     "Wavefront OBJ mesh (*.obj)");
    if (path.isEmpty()) return;
    QSaveFile output(path);
    if (!output.open(QIODevice::WriteOnly)) { status_->setText(output.errorString()); return; }
    const int pixels = depthWidth_ * depthHeight_;
    std::vector<int> vertexId(size_t(pixels), 0);
    std::vector<float> depths(size_t(pixels), 0);
    int vertices = 0, faces = 0;
    for (int y = 0; y < depthHeight_; ++y) for (int x = 0; x < depthWidth_; ++x) {
      float point[3];
      if (!depthPoint(x, y, point)) continue;
      const int i = y * depthWidth_ + x;
      vertexId[size_t(i)] = ++vertices;
      depths[size_t(i)] = point[2];
      const QByteArray line = "v " + QByteArray::number(point[0], 'g', 8) + " " +
          QByteArray::number(point[1], 'g', 8) + " " + QByteArray::number(point[2], 'g', 8) + "\n";
      if (output.write(line) != line.size()) { status_->setText(output.errorString()); return; }
    }
    auto face = [&](int a, int b, int c) {
      if (!vertexId[size_t(a)] || !vertexId[size_t(b)] || !vertexId[size_t(c)]) return true;
      const float z0 = depths[size_t(a)], z1 = depths[size_t(b)], z2 = depths[size_t(c)];
      const float limit = std::max(0.03f, std::min({z0, z1, z2}) * 0.05f);
      if (std::max({z0, z1, z2}) - std::min({z0, z1, z2}) > limit) return true;
      const QByteArray line = "f " + QByteArray::number(vertexId[size_t(a)]) + " " +
          QByteArray::number(vertexId[size_t(b)]) + " " +
          QByteArray::number(vertexId[size_t(c)]) + "\n";
      if (output.write(line) != line.size()) return false;
      ++faces;
      return true;
    };
    for (int y = 0; y + 1 < depthHeight_; ++y) for (int x = 0; x + 1 < depthWidth_; ++x) {
      const int a = y * depthWidth_ + x, b = a + 1, c = a + depthWidth_, d = c + 1;
      if (!face(a, c, b) || !face(b, c, d)) { status_->setText(output.errorString()); return; }
    }
    status_->setText(output.commit() ? QString("Exported %1 vertices, %2 faces").arg(vertices).arg(faces)
                                      : output.errorString());
  }
  void stopStream() {
    if (recorder_.active()) {
      if (!recorder_.stop()) status_->setText(recorder_.error());
      recordButton_->setText("Record video");
    }
    if (!worker_) return;
    worker_->requestInterruption();
    worker_->wait();
    delete worker_; worker_ = nullptr;
  }
  void toggleRecording() {
    if (recorder_.active()) {
      if (!recorder_.stop()) status_->setText(recorder_.error());
      recordButton_->setText("Record video"); return;
    }
    if (!worker_ || lastImage_.isNull()) { status_->setText("Start a stream before recording"); return; }
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::MoviesLocation);
    const QString suggested = folder + "/Kinect-" +
        QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss") + ".mp4";
    const QString path = QFileDialog::getSaveFileName(this, "Record Kinect video", suggested,
                                                     "MP4 video (*.mp4)");
    if (path.isEmpty()) return;
    if (!recorder_.start(path, lastImage_.width(), lastImage_.height())) {
      status_->setText(recorder_.error()); return;
    }
    recordButton_->setText("Stop recording");
  }
private:
  bool depthPoint(int x, int y, float (&point)[3]) const {
    const quint16 rawMm = u16(depth_.constData() + (y * depthWidth_ + x) * 2);
    const quint16 millimeters = depthCalibration_.stationCount() > 0 ?
        depthCalibration_.correct(x, y, rawMm) : rawMm;
    if (!millimeters) return false;
    const bool distortion = depthExtension_.size() >= sizeof(KinectOneRemoldProtocol::ReplyExtension) &&
        (u32(depthExtension_.constData() + 12) & KinectOneRemoldProtocol::ReplyExtensionDepthDistortion);
    auto coefficient = [&](int offset) {
      float value = 0;
      if (distortion) std::memcpy(&value, depthExtension_.constData() + offset, sizeof(value));
      return double(value);
    };
    const double center = distortion ? coefficient(36) : 0.5;
    const double dx = (x + center - depthCx_) / depthFx_;
    const double dy = (y + center - depthCy_) / depthFy_;
    double ux = dx, uy = dy;
    const double k1 = coefficient(16), k2 = coefficient(20), k3 = coefficient(24);
    const double p1 = coefficient(28), p2 = coefficient(32);
    for (int n = 0; distortion && n < 20; ++n) {
      const double xx = ux * ux, yy = uy * uy, rr = xx + yy;
      const double radial = 1 + k1 * rr + k2 * rr * rr + k3 * rr * rr * rr;
      const double ex = dx - (ux * radial + 2 * p1 * ux * uy + p2 * (rr + 2 * xx));
      const double ey = dy - (uy * radial + p1 * (rr + 2 * yy) + 2 * p2 * ux * uy);
      ux += ex; uy += ey;
      if (ex * ex + ey * ey < 1e-20) break;
    }
    if (!std::isfinite(ux) || !std::isfinite(uy) || std::abs(ux) > 4 || std::abs(uy) > 4) {
      ux = dx; uy = dy;
    }
    const float z = float(millimeters) * 0.001f;
    point[0] = float(ux * z); point[1] = float(uy * z); point[2] = z;
    return true;
  }
  QComboBox* devices_ = nullptr; QComboBox* modes_ = nullptr;
  QString preferredId_;
  QComboBox* led_ = nullptr;
  QSpinBox* tilt_ = nullptr;
  QLabel* preview_ = nullptr; QLabel* status_ = nullptr;
  std::vector<Device> found_; StreamWorker* worker_ = nullptr; QImage lastImage_;
  QByteArray depth_, depthExtension_, poseJson_;
  int depthWidth_ = 0, depthHeight_ = 0;
  double depthFx_ = 0, depthFy_ = 0, depthCx_ = 0, depthCy_ = 0;
  QPushButton* recordButton_ = nullptr;
  VideoRecorder recorder_;
  DepthCalibrationProfile depthCalibration_;
  bool calibrationLoaded_ = false;
};

} // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
#ifdef Q_OS_MACOS
  BundledUsbHost nativeUsbHost;
  const QStringList startupArguments=app.arguments();
  const bool diagnostic=std::any_of(startupArguments.cbegin(),startupArguments.cend(),
      [](const QString& arg){return arg.startsWith("--check-");});
  const bool liveDiagnostic=std::any_of(startupArguments.cbegin(),startupArguments.cend(),
      [](const QString& arg){return arg.startsWith("--check-live-")||arg=="--check-stream-handoff";});
  if(!diagnostic||liveDiagnostic)startBundledUsbHost(nativeUsbHost);
#endif
  if (app.arguments().contains("--check-stream-handoff")) {
    auto sensors=discoverAt(sdkPath(true));
    sensors.erase(std::remove_if(sensors.begin(),sensors.end(),[](const Device& d){
      return d.state!="Ready"||d.camera.isEmpty();
    }),sensors.end());
    if(sensors.empty())return 2;
    StreamWorker first(sensors.front(),1u,0),second(sensors.front(),5u,0);
    int firstFrames=0,secondFrames=0;
    QObject::connect(&first,&StreamWorker::frameReady,&app,[&](const QImage&){++firstFrames;});
    QObject::connect(&second,&StreamWorker::frameReady,&app,[&](const QImage&){++secondFrames;});
    first.startLatest();QElapsedTimer timer;timer.start();
    while(firstFrames<2&&timer.elapsed()<15000) {app.processEvents();QThread::msleep(10);}
    if(firstFrames<2) {first.requestInterruption();first.wait();return 3;}
    second.startLatest();timer.restart();
    while(secondFrames<2&&timer.elapsed()<15000) {app.processEvents();QThread::msleep(10);}
    const bool passed=!first.isRunning()&&secondFrames>=2;
    second.requestInterruption();second.wait();
    fprintf(stderr,"Stream handoff: first %d frames, second %d frames, previous stopped %d\n",
            firstFrames,secondFrames,!first.isRunning());
    return passed?0:4;
  }
  if (app.arguments().contains("--check-mocap-smoothing")) {
    MocapPoseSmoother smoother;
    auto sample=[&](double t,double x) {
      const QJsonObject joint{{"p",QJsonArray{x,0.,2.}},{"uv",QJsonArray{x,.5}},{"c",.8}};
      const QJsonObject root{{"device","test"},{"t",t},
          {"bodies",QJsonArray{QJsonObject{{"id",1},
              {"joints",QJsonObject{{"wristL",joint}}}}}}};
      const QJsonObject filtered=smoother.filter(root);
      return filtered.value("bodies").toArray().first().toObject()
          .value("joints").toObject().value("wristL").toObject()
          .value("p").toArray().first().toDouble();
    };
    double rawVariation=0,filteredVariation=0,previous=0;
    for(int i=0;i<100;++i) {
      const double noisy=(i%2?.025:-.025);
      const double value=sample(1.+i/30.,noisy);
      rawVariation+=std::abs(noisy);
      filteredVariation+=std::abs(value);
      previous=value;
    }
    const double spike=sample(1.+100./30.,.8);
    if(filteredVariation>=rawVariation*.55||spike-previous>.15)return 2;
    double following=spike;
    for(int i=1;i<=30;++i)following=sample(1.+(100+i)/30.,double(i)/30.);
    fprintf(stderr,"Smoothing: still ratio %.3f, spike %.3f, motion %.3f\n",
            filteredVariation/rawVariation,spike-previous,following);
    return following>.75?0:3;
  }
  if (app.arguments().contains("--check-clip-editor")) {
    QTemporaryDir dir;if(!dir.isValid())return 2;
    const QString bodyPath=dir.path()+"/body.jsonl",facePath=dir.path()+"/face.jsonl";
    QFile body(bodyPath),face(facePath);
    if(!body.open(QIODevice::WriteOnly)||!face.open(QIODevice::WriteOnly))return 3;
    for(int i=0;i<120;++i) {
      const double x=i*.01+(i%2?.02:-.02);
      const int phase=i%30;
      const double leftAnkleX=(i/30)*.3+(phase<15?0.:(phase-15)*.02);
      const double rightAnkleX=((i+15)/30)*.3+(phase>=15?0.:phase*.02);
      const double leftFootY=phase<15?1.:.92;
      const double rightFootY=phase>=15?1.:.92;
      const QJsonObject joints{{"hipL",QJsonObject{{"p",QJsonArray{x-.1,0.,2.}}, {"c",.9}}},
                               {"hipR",QJsonObject{{"p",QJsonArray{x+.1,0.,2.}}, {"c",.9}}},
                               {"shoulderL",QJsonObject{{"p",QJsonArray{x-.2,-.4,2.}}, {"c",.9}}},
                               {"shoulderR",QJsonObject{{"p",QJsonArray{x+.2,-.4,2.}}, {"c",.9}}},
                               {"wristL",QJsonObject{{"p",QJsonArray{x-.35,.05,2.}}, {"c",.9}}},
                               {"wristR",QJsonObject{{"p",QJsonArray{x+.35,.05,2.}}, {"c",.9}}},
                               {"kneeL",QJsonObject{{"p",QJsonArray{(x-.1+leftAnkleX)*.5,.5,2.}}, {"c",.9}}},
                               {"kneeR",QJsonObject{{"p",QJsonArray{(x+.1+rightAnkleX)*.5,.5,2.}}, {"c",.9}}},
                               {"ankleL",QJsonObject{{"p",QJsonArray{leftAnkleX,leftFootY,2.}}, {"c",.9}}},
                               {"ankleR",QJsonObject{{"p",QJsonArray{rightAnkleX,rightFootY,2.}}, {"c",.9}}},
                               {"head",QJsonObject{{"p",QJsonArray{x,-.7,2.}}, {"c",.9}}},
                               {"nose",QJsonObject{{"uv",QJsonArray{.5,.2}}, {"c",.9}}},
                               {"earL",QJsonObject{{"uv",QJsonArray{.4,.2}}, {"c",.9}}},
                               {"earR",QJsonObject{{"uv",QJsonArray{.6,.2}}, {"c",.9}}}};
      const QJsonObject frame{{"t",1.+i/30.},{"tracked",true},
          {"bodies",QJsonArray{QJsonObject{{"joints",joints}}}}};
      body.write(QJsonDocument(frame).toJson(QJsonDocument::Compact)+'\n');
      if(i%2==0) {
        QJsonArray landmarks;
        for(int point=0;point<478;++point)landmarks.append(QJsonArray{.5,.45,0.});
        landmarks[234]=QJsonArray{.3,.5,0.};landmarks[454]=QJsonArray{.7,.5,0.};
        const QJsonObject facial{{"t",5.+i/30.},
            {"face",QJsonObject{{"tracked",true},
                {"landmarks",landmarks},
                {"blendshapes",QJsonObject{{"jawOpen",double(i)/120.}}}}}};
        face.write(QJsonDocument(facial).toJson(QJsonDocument::Compact)+'\n');
      }
    }
    body.close();face.close();
    auto compact=[&](const QString& input,const QString& output) {
      QFile src(input),dst(output);
      if(!src.open(QIODevice::ReadOnly)||!dst.open(QIODevice::WriteOnly))return false;
      dst.write("RMC1",4);
      while(!src.atEnd()) {
        const QByteArray packed=qCompress(src.readLine().trimmed(),3);
        const quint32 n=quint32(packed.size());QByteArray prefix(4,0);
        for(int i=0;i<4;++i)prefix[i]=char((n>>(i*8))&255);
        if(dst.write(prefix)!=4||dst.write(packed)!=packed.size())return false;
      }
      return true;
    };
    const QString compactBody=dir.path()+"/body.rmclip",compactFace=dir.path()+"/face.rmclip";
    if(!compact(bodyPath,compactBody)||!compact(facePath,compactFace))return 9;
    const auto indexed=MocapClipIndex::build(compactBody);
    QFile preview(compactBody);
    if(!indexed.error.isEmpty()||indexed.frames!=120||!preview.open(QIODevice::ReadOnly)||
       indexed.frameAt(preview,75).value("t").toDouble()<3.4)return 12;
    MocapClipOptions options;
    options.source=compactBody;options.face=compactFace;options.output=dir.path()+"/out.jsonl";
    options.trimStart=.5;options.trimEnd=3.5;options.loopBlend=.3;
    options.smoothing=60;options.inPlaceLoop=true;
    options.bvhOutput=dir.path()+"/out.bvh";
    const auto processed=MocapClipProcessor::process(options);
    if(!processed.error.isEmpty()) {fprintf(stderr,"Clip editor: %s\n",qPrintable(processed.error));return 4;}
    QFile out(options.output);if(!out.open(QIODevice::ReadOnly))return 5;
    QJsonObject first,last;int count=0;double maxPlantedStep=0,previousAnkleX=0;
    int previousInputFrame=-100;
    while(!out.atEnd()) {
      const auto frame=QJsonDocument::fromJson(out.readLine()).object();
      if(frame.isEmpty()||!frame.value("face").isObject())return 6;
      if(frame.value("face").toObject().value("registration").toObject().value("state")!="aligned")return 14;
      const QJsonArray mappedNose=frame.value("face").toObject()
          .value("landmarks").toArray()[1].toArray();
      if(mappedNose.size()!=3||std::abs(mappedNose[0].toDouble()-.5)>.03||
         std::abs(mappedNose[1].toDouble()-.2)>.03)return 16;
      const int inputFrame=int(std::lround((frame.value("t").toDouble()-1.)*30));
      const QJsonArray left=frame.value("bodies").toArray()[0].toObject()
          .value("joints").toObject().value("ankleL").toObject().value("p").toArray();
      if(left.size()==3) {
        const double ankleX=left[0].toDouble();
        if(inputFrame==previousInputFrame+1&&inputFrame%30>2&&inputFrame%30<15)
          maxPlantedStep=std::max(maxPlantedStep,std::abs(ankleX-previousAnkleX));
        previousAnkleX=ankleX;previousInputFrame=inputFrame;
      }
      if(count++==0)first=frame;last=frame;
    }
    auto x=[&](const QJsonObject& frame){return frame.value("bodies").toArray()[0].toObject()
        .value("joints").toObject().value("hipL").toObject().value("p").toArray()[0].toDouble();};
    fprintf(stderr,"Clip editor: %d frames, seam %.4f m, score %.4f, planted step %.4f m, held %d\n",
        count,std::abs(x(first)-x(last)),processed.seamScore,maxPlantedStep,
        processed.plantedFootFrames);
    QFile bvh(options.bvhOutput);if(!bvh.open(QIODevice::ReadOnly))return 8;
    if(count!=processed.frames||count<=50||processed.seamScore>=1e8||
       processed.plantedFootFrames<5||
       maxPlantedStep>=.025||std::abs(x(first)-x(last))>=.08||
       !bvh.read(32).startsWith("HIERARCHY\n"))return 7;
    options.output=dir.path()+"/out.rmclip";
    options.bvhOutput=dir.path()+"/compact-out.bvh";
    const auto compactResult=MocapClipProcessor::process(options);
    const auto compactIndex=MocapClipIndex::build(options.output);
    QFile compactPreview(options.output);
    if(!compactResult.error.isEmpty()||!compactIndex.error.isEmpty()||
       compactIndex.frames!=count||!compactPreview.open(QIODevice::ReadOnly)||
       !compactIndex.frameAt(compactPreview,20).value("face").isObject())return 13;
    QFile damaged(compactBody);
    if(!damaged.open(QIODevice::ReadWrite)||!damaged.resize(damaged.size()-10))return 10;
    damaged.close();options.output=dir.path()+"/damaged.jsonl";options.bvhOutput.clear();
    options.trimEnd=0;
    if(MocapClipProcessor::process(options).error.isEmpty())return 11;
    const QString replay=qEnvironmentVariable("REMOLD_CLIP_TEST_SOURCE");
    if(!replay.isEmpty()) {
      MocapClipOptions real;real.source=replay;real.output=dir.path()+"/replay.rmclip";
      real.smoothing=35;real.loopBlend=.4;real.inPlaceLoop=true;
      const auto outcome=MocapClipProcessor::process(real);
      const auto index=MocapClipIndex::build(real.output);
      if(!outcome.error.isEmpty()||!index.error.isEmpty()||index.frames!=outcome.frames||
         outcome.frames<30||outcome.seamScore>=1e8)return 15;
      QFile replayFile(real.output);
      if(!replayFile.open(QIODevice::ReadOnly))return 17;
      const QJsonObject opening=index.frameAt(replayFile,0),closing=index.frameAt(replayFile,index.frames-1);
      auto hip=[&](const QJsonObject& frame){
        const QJsonArray bodies=frame.value("bodies").toArray();
        if(bodies.isEmpty())return QVector3D{};
        const QJsonObject joints=bodies[0].toObject().value("joints").toObject();
        const QJsonArray l=joints.value("hipL").toObject().value("p").toArray();
        const QJsonArray r=joints.value("hipR").toObject().value("p").toArray();
        return l.size()==3&&r.size()==3?QVector3D(float((l[0].toDouble()+r[0].toDouble())*.5),
            float((l[1].toDouble()+r[1].toDouble())*.5),float((l[2].toDouble()+r[2].toDouble())*.5)):
            QVector3D{};
      };
      const double gap=(hip(opening)-hip(closing)).length();
      fprintf(stderr,"Real clip replay: %d frames, seam score %.4f, held %d, error %s\n",
          outcome.frames,outcome.seamScore,outcome.plantedFootFrames,qPrintable(outcome.error));
      fprintf(stderr,"Real clip loop hip gap: %.4f m\n",gap);
      if(gap>.12)return 15;
      const QString faceReplay=qEnvironmentVariable("REMOLD_CLIP_TEST_FACE");
      if(!faceReplay.isEmpty()) {
        MocapClipOptions composite;
        composite.source=replay;composite.face=faceReplay;
        composite.output=qEnvironmentVariable("REMOLD_CLIP_TEST_OUTPUT");
        if(composite.output.isEmpty())composite.output=dir.path()+"/face-composite.rmclip";
        const auto attached=MocapClipProcessor::process(composite);
        const auto faceIndex=MocapClipIndex::build(composite.output);
        if(!attached.error.isEmpty()||!faceIndex.error.isEmpty()||faceIndex.frames<30)return 18;
        QFile faceFile(composite.output);if(!faceFile.open(QIODevice::ReadOnly))return 19;
        int aligned=0,expressions=0;
        for(int i=0;i<faceIndex.frames;++i) {
          const QJsonObject face=faceIndex.frameAt(faceFile,i).value("face").toObject();
          if(face.value("registration").toObject().value("state").toString().startsWith("aligned"))++aligned;
          if(face.value("blendshapes").isObject())++expressions;
        }
        fprintf(stderr,"Real face composite: %d aligned meshes, %d expression frames\n",
            aligned,expressions);
        if(expressions<30)return 20;
        MocapClipOptions faceLoop;
        faceLoop.source=faceReplay;faceLoop.output=dir.path()+"/face-loop.rmclip";
        faceLoop.loopBlend=.4;faceLoop.inPlaceLoop=true;
        const auto loopedFace=MocapClipProcessor::process(faceLoop);
        if(!loopedFace.error.isEmpty()||loopedFace.frames<30||loopedFace.warning.isEmpty())return 21;
      }
    }
    return 0;
  }
  if (app.arguments().contains("--check-articulated-fit")) {
    ArticulatedPoseFit fit;
    auto frame=[&](int index,double elbowNoise) {
      QJsonObject joints;
      const char* names[]{"shoulderL","shoulderR","elbowL","elbowR",
          "wristL","wristR","hipL","hipR","kneeL","kneeR","ankleL","ankleR"};
      const QVector3D points[]{
          {-.2f,0,2},{.2f,0,2},{-.2f,.3f,2},{.2f,.3f,2},
          {-.2f,.57f,2},{.2f,.57f,2},{-.15f,.48f,2},{.15f,.48f,2},
          {-.15f,.9f,2},{.15f,.9f,2},{-.15f,1.32f,2},{.15f,1.32f,2}};
      for(int i=0;i<12;++i) {
        QVector3D p=points[i];if(i==2)p.setY(p.y()+float(elbowNoise));
        joints[names[i]]=QJsonObject{{"p",QJsonArray{p.x(),p.y(),p.z()}},
            {"uv",QJsonArray{.5,.5}},{"c",.9}};
      }
      return fit.fit(QJsonObject{{"device","test"},{"t",1.+index/30.},
          {"bodies",QJsonArray{QJsonObject{{"id",1},{"joints",joints}}}}});
    };
    for(int i=0;i<35;++i)frame(i,0);
    const QJsonObject fitted=frame(35,.12).value("bodies").toArray()[0].toObject()
        .value("joints").toObject();
    auto pos=[&](const char* name){const QJsonArray p=fitted.value(name).toObject().value("p").toArray();
      return QVector3D(p[0].toDouble(),p[1].toDouble(),p[2].toDouble());};
    const float upper=(pos("shoulderL")-pos("elbowL")).length();
    const float lower=(pos("elbowL")-pos("wristL")).length();
    fprintf(stderr,"Articulated fit: upper %.4f lower %.4f\n",upper,lower);
    return std::abs(upper-.30f)<.025f&&std::abs(lower-.27f)<.025f?0:2;
  }
  if (app.arguments().contains("--check-mocap-rig")) {
    QJsonObject joints;
    auto add=[&](const char* name,float x,float y,float z){
      joints[QLatin1String(name)]=QJsonObject{{"p",QJsonArray{x,y,z}},{"c",.9},{"s",2}};
    };
    add("head",0,-.8f,2);add("shoulderL",.2f,-.5f,2);
    add("shoulderR",-.2f,-.5f,2);add("elbowL",.5f,-.5f,2);
    add("elbowR",-.5f,-.5f,2);add("wristL",.8f,-.5f,2);
    add("wristR",-.8f,-.5f,2);add("hipL",.15f,0,2);
    add("hipR",-.15f,0,2);add("kneeL",.15f,.45f,2);
    add("kneeR",-.15f,.45f,2);add("ankleL",.15f,.9f,2);
    add("ankleR",-.15f,.9f,2);
    const QByteArray camera=QJsonDocument(QJsonObject{{"v",1},{"device","synthetic"},
        {"t",1.0},{"frame",1},{"space","camera"},{"tracked",true},
        {"bodies",QJsonArray{QJsonObject{{"id",1},{"tracked",true},
            {"confidence",.9},{"joints",joints}}}}}).toJson(QJsonDocument::Compact);
    MocapRig rig;const HumanoidPose pose=rig.retarget(camera);
    if(!pose.tracked||pose.bones[0].position.y()<.8f) return 2;
    QJsonArray tiltedPoints;
    for(int i=0;i<478;++i)tiltedPoints.append(QJsonArray{.5,.5,0.});
    tiltedPoints[33]=QJsonArray{.4,.45,0.};
    tiltedPoints[263]=QJsonArray{.6,.55,0.};
    QJsonObject tiltedRoot=QJsonDocument::fromJson(camera).object();
    tiltedRoot["face"]=QJsonObject{{"tracked",true},{"confidence",.9},
                                    {"landmarks",tiltedPoints}};
    MocapRig headRig;
    const HumanoidPose tilted=headRig.retarget(QJsonDocument(tiltedRoot).toJson());
    if(!tilted.tracked||tilted.bones[4].localRotation.z()<.05f)return 16;
    QJsonObject earJoints=joints;
    earJoints["earL"]=QJsonObject{{"p",QJsonArray{.1,-.82,2.}}, {"c",.9}};
    earJoints["earR"]=QJsonObject{{"p",QJsonArray{-.1,-.72,2.}}, {"c",.9}};
    QJsonObject earRoot=QJsonDocument::fromJson(camera).object();
    earRoot["bodies"]=QJsonArray{QJsonObject{{"id",1},{"joints",earJoints}}};
    MocapRig bodyHeadRig;
    const HumanoidPose bodyTilt=bodyHeadRig.retarget(QJsonDocument(earRoot).toJson());
    if(!bodyTilt.tracked||bodyTilt.bones[4].localRotation.z()>.0f)return 18;
    add("wristL",.65f,-.2f,1.8f);
    add("handL_8",.8f,-.1f,1.7f);
    add("handL_4",.68f,-.1f,1.9f);
    const QByteArray bentCamera=QJsonDocument(QJsonObject{{"v",1},{"device","synthetic"},
        {"t",1.033},{"frame",2},{"space","camera"},{"tracked",true},
        {"bodies",QJsonArray{QJsonObject{{"id",1},{"tracked",true},
            {"confidence",.9},{"joints",joints}}}}}).toJson(QJsonDocument::Compact);
    const HumanoidPose bent=rig.retarget(bentCamera);
    if(!bent.tracked||!std::isfinite(bent.bones[6].rotation.scalar())||
       std::abs(bent.bones[6].rotation.length()-1.f)>.001f||
       (bent.bones[6].rotation-pose.bones[6].rotation).length()<.02f||
       bent.bones[8].confidence<=0||
       std::abs(bent.bones[8].position.x()+.65f)>.01f) return 11;
    QJsonObject missingJoints=joints;missingJoints.remove("ankleL");
    const QByteArray missingCamera=QJsonDocument(QJsonObject{{"device","synthetic"},
        {"t",1.066},{"frame",3},{"tracked",true},
        {"bodies",QJsonArray{QJsonObject{{"id",1},{"joints",missingJoints}}}}}).toJson();
    const HumanoidPose held=rig.retarget(missingCamera);
    if(!held.tracked||(held.bones[15].position-bent.bones[15].position).length()>.02f)
      return 17;
    MocapRig mirrorRig;const HumanoidPose mirrored=mirrorRig.retarget(camera,true);
    if(!mirrored.tracked||std::abs(mirrored.bones[8].position.x()+
                                  pose.bones[12].position.x())>.02f) return 10;
    const QByteArray vmc=MocapNetwork::encodeVmc(pose);
    if(!vmc.startsWith("#bundle")||!vmc.contains("/VMC/Ext/Bone/Pos")||
       !vmc.contains("LeftUpperArm")) return 3;
    QJsonObject coefficients{{"jawOpen",.7}};
    for(int i=0;i<50;++i)coefficients[QString("test%1").arg(i)]=0.;
    const QJsonObject face{{"tracked",true},{"blendshapes",coefficients}};
    if(!MocapNetwork::encodeFaceCap(face).contains("/W")||
       !MocapNetwork::encodeVmcFace(face).contains("/VMC/Ext/Blend/Apply")||
       !MocapNetwork::encodeVmcFace(face).contains("jawOpen"))return 12;
    QUdpSocket faceReceiver;
    if(!faceReceiver.bind(QHostAddress::LocalHost,0))return 13;
    MocapNetwork faceNetwork;faceNetwork.setJsonEnabled(true);
    faceNetwork.setJsonTarget(QHostAddress::LocalHost,faceReceiver.localPort());
    QJsonArray facePoints;for(int i=0;i<478;++i)
      facePoints.append(QJsonArray{.5,.4,.01});
    QJsonObject capturedFace=face;capturedFace["landmarks"]=facePoints;
    QJsonObject withFace=QJsonDocument::fromJson(camera).object();
    withFace["face"]=capturedFace;
    if(!faceNetwork.send(camera,&pose,QJsonDocument(withFace).toJson(QJsonDocument::Compact)))return 14;
    QByteArray faceDatagram;
    for(int i=0;i<2;++i) {
      if(!faceReceiver.waitForReadyRead(1000))return 15;
      QByteArray packet(int(faceReceiver.pendingDatagramSize()),0);
      faceReceiver.readDatagram(packet.data(),packet.size());
      if(QJsonDocument::fromJson(packet).object().value("kind")=="face-landmarks")faceDatagram=packet;
    }
    const QJsonObject compactFace=QJsonDocument::fromJson(faceDatagram).object();
    if(compactFace.value("blendshapes").toObject().value("jawOpen").toDouble()!=.7||
       QByteArray::fromBase64(compactFace.value("data").toString().toLatin1()).size()!=478*6)return 16;
    const QJsonObject unity=MocapRig::unityJson(camera,pose);
    if(unity.value("space")!="unity"||unity.value("bodies").toArray().isEmpty()) return 4;
    QTemporaryDir directory;if(!directory.isValid()) return 5;
    const QString path=directory.path()+"/test.bvh";
    MocapBvh bvh;if(!bvh.start(path)) return 6;
    bvh.append(pose);if(!bvh.stop()) return 7;
    QFile file(path);if(!file.open(QIODevice::ReadOnly)) return 8;
    const QByteArray data=file.readAll();
    return data.startsWith("HIERARCHY\n")&&
        data.contains("Frames: 00000000000000000001\n")&&
        data.contains("JOINT LeftUpperArm")&&
        data.contains("OFFSET -30.0000 0.0000 0.0000")?0:9;
  }
  if (app.arguments().contains("--check-factory-registration")) {
    KinectOneRemoldProtocol::ReplyExtensionV2 extension{};
    extension.base.flags=7;extension.base.colorFx=1000;
    extension.base.colorFy=1000;extension.base.colorCx=960;
    extension.base.colorCy=540;extension.base.colorShiftD=1000;
    extension.base.colorShiftM=50;
    extension.colorPolynomial[7]=.60246575f;
    extension.colorPolynomial[18]=.60246575f;
    DepthIntrinsics camera{512,424,365,365,255.5,211.5,
        QByteArray(reinterpret_cast<const char*>(&extension),sizeof(extension))};
    float u=0,v=0;
    if(!RgbdRegistration::projectKinectOne(camera,255,211,1000,1920,1080,u,v)||
       std::abs(u-960)>1||std::abs(v-540)>1)return 2;
    if(!RgbdRegistration::projectKinectOne(camera,255,211,2000,1920,1080,u,v)||
       std::abs(u-935)>1)return 3;
    if(!RgbdRegistration::projectKinectOne(camera,355,211,1000,1920,1080,u,v)||
       std::abs(u-1234)>2)return 4;
    return 0;
  }
  if (app.arguments().contains("--check-mocap-stage")) {
    MocapStageView stage;stage.resize(1000,520);
    QImage frame(640,480,QImage::Format_RGB32);frame.fill(QColor(48,57,70));
    stage.setFrame(frame);
    QJsonObject joints;
    auto add=[&](const char* name,double x,double y,double z,double u,double v){
      joints[QLatin1String(name)]=QJsonObject{{"p",QJsonArray{x,y,z}},
          {"uv",QJsonArray{u,v}},{"c",.9}};
    };
    add("head",0,-.8,2,.5,.2);add("shoulderL",.25,-.5,2,.6,.35);
    add("shoulderR",-.25,-.5,2,.4,.35);add("elbowL",.5,-.3,2,.7,.45);
    add("elbowR",-.5,-.3,2,.3,.45);add("wristL",.7,-.1,2,.8,.55);
    add("wristR",-.7,-.1,2,.2,.55);add("hipL",.2,0,2,.6,.65);
    add("hipR",-.2,0,2,.4,.65);add("kneeL",.2,.5,2,.6,.8);
    add("kneeR",-.2,.5,2,.4,.8);add("ankleL",.2,.9,2,.6,.95);
    add("ankleR",-.2,.9,2,.4,.95);
    QJsonArray facePoints;
    for(int i=0;i<478;++i)facePoints.append(QJsonArray{.5,.4,0.});
    static constexpr int oval[]{10,338,297,332,284,251,389,356,454,323,361,288,
        397,365,379,378,400,377,152,148,176,149,150,136,172,58,132,93,234,
        127,162,21,54,103,67,109};
    for(size_t i=0;i<std::size(oval);++i) {
      const double angle=double(i)*6.283185307/std::size(oval)-1.570796327;
      facePoints[oval[i]]=QJsonArray{.5+.16*std::cos(angle),.4+.22*std::sin(angle),0.};
    }
    stage.setPose(QJsonDocument(QJsonObject{{"bodies",QJsonArray{QJsonObject{{"joints",joints}}}},
        {"face",QJsonObject{{"tracked",true},{"landmarks",facePoints}}}}).toJson());
    stage.show();app.processEvents();
    const QImage rendered=stage.grab().toImage();
    rendered.save("/tmp/kinect-remold-mocap-stage.png");
    int green=0;for(int y=0;y<rendered.height();++y)for(int x=0;x<rendered.width();++x){
      const QColor c=rendered.pixelColor(x,y);
      if(c.green()>c.red()*1.5&&c.green()>c.blue()*1.15)++green;
    }
    int filled=0;for(int y=0;y<rendered.height();++y)for(int x=0;x<rendered.width();++x){
      const QColor c=rendered.pixelColor(x,y);
      if(c.green()>c.red()*1.25&&c.blue()>c.red()*1.3&&c.green()>95)++filled;
    }
    return green>100&&filled>1000?0:2;
  }
  if (app.arguments().contains("--check-mocap-window")) {
    MotionCaptureWindow window;window.show();app.processEvents();
    const QImage image=window.grab().toImage();
    image.save("/tmp/kinect-remold-mocap-window.png");
    return image.width()>=1100&&image.height()>=700?0:2;
  }
  if (app.arguments().contains("--check-mocap-guidance")) {
    MotionCaptureWindow window;window.show();app.processEvents();
    const int result=window.checkGuidance();
    app.processEvents();window.grab().save("/tmp/kinect-remold-mocap-guidance.png");
    return result;
  }
  if (app.arguments().contains("--check-live-face-framing")) {
    MotionCaptureWindow window;
    return window.checkLiveFraming(true);
  }
  if (app.arguments().contains("--check-live-body-framing")) {
    MotionCaptureWindow window;
    return window.checkLiveFraming(false);
  }
  if (app.arguments().contains("--check-lan-pose")) {
    const QStringList args=app.arguments();
    const int index=args.indexOf("--check-lan-pose");
    if(index+3>=args.size())return 2;
    LanPoseLink host,client;
    if(!host.startHost(0,args[index+1],args[index+2],args[index+3])) {
      fprintf(stderr,"LAN host: %s\n",host.error().toUtf8().constData());return 3;
    }
    QByteArray received,atController;
    QObject::connect(&client,&LanPoseLink::remotePose,&app,
                     [&](const QString&,const QByteArray& pose){received=pose;});
    QObject::connect(&host,&LanPoseLink::remotePose,&app,
                     [&](const QString&,const QByteArray& pose){atController=pose;});
    QObject::connect(&client,&LanPoseLink::failed,&app,
                     [&](const QString& error){fprintf(stderr,"LAN client: %s\n",error.toUtf8().constData());});
    if(!client.connectPeer(QUrl(QString("wss://localhost:%1/").arg(host.port())),
                           args[index+1],args[index+2],args[index+3])) return 4;
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<5000&&host.connectedPeers()==0) {app.processEvents();QThread::msleep(5);}
    if(host.connectedPeers()==0)return 5;
    const QJsonObject joints{{"head",QJsonObject{{"p",QJsonArray{0.,-.8,2.}},
                                                    {"c",.9}}}};
    const QJsonObject body{{"id",1},{"tracked",true},{"joints",joints}};
    const QByteArray pose=QJsonDocument(QJsonObject{{"v",1},{"device","synthetic"},
        {"space","camera"},{"bodies",QJsonArray{body}}}).toJson();
    host.broadcastPose(pose);
    timer.restart();
    while(timer.elapsed()<3000&&received.isEmpty()) {app.processEvents();QThread::msleep(5);}
    client.broadcastPose(pose);
    timer.restart();
    while(timer.elapsed()<3000&&atController.isEmpty()) {app.processEvents();QThread::msleep(5);}
    const QJsonObject remote=QJsonDocument::fromJson(received).object();
    const QJsonObject controller=QJsonDocument::fromJson(atController).object();
    return remote.value("device").toString().endsWith("/synthetic")&&
        controller.value("device").toString().endsWith("/synthetic")&&
        remote.value("seq").toInt()==1&&
        !remote.value("tMonoNs").toString().isEmpty()&&
        remote.value("clockUncertaintyNs").toDouble()==1000000000.0&&
        remote.value("calibrationRevision").toString()=="unconfigured"?0:6;
  }
  if (app.arguments().contains("--check-websocket")) {
    MocapWebSocket server;
    if (!server.start(0)) return 2;
    QTcpSocket client;
    client.connectToHost(QHostAddress::LocalHost,server.port());
    if (!client.waitForConnected(2000)) return 3;
    client.write("GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                 "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                 "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n");
    client.flush();
    QElapsedTimer timer;timer.start();
    while (timer.elapsed()<2000 && !client.bytesAvailable()) {app.processEvents();QThread::msleep(5);}
    const QByteArray reply=client.readAll();
    if (!reply.startsWith("HTTP/1.1 101") ||
        !reply.contains("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")) return 4;
    server.broadcast("{\"v\":1}");
    timer.restart();
    while (timer.elapsed()<2000 && client.bytesAvailable()<9) {app.processEvents();QThread::msleep(5);}
    const QByteArray frame=client.readAll();
    return frame==QByteArray("\x81\x07{\"v\":1}",9)?0:5;
  }
  if (app.arguments().contains("--check-video-recorder")) {
    QTemporaryDir directory;
    if (!directory.isValid()) return 2;
    const QString path = directory.path() + "/recorder.mp4";
    VideoRecorder recorder;
    if (!recorder.start(path, 320, 240)) {
      fprintf(stderr, "Video recorder: %s\n", recorder.error().toUtf8().constData()); return 3;
    }
    for (int number = 0; number < 45; ++number) {
      QImage frame(320, 240, QImage::Format_RGB32);
      frame.fill(QColor::fromHsv(number * 8 % 360, 255, 200));
      if (!recorder.append(frame)) return 4;
    }
    if (!recorder.stop()) return 5;
    return QFileInfo(path).size() > 1024 ? 0 : 6;
  }
  if (app.arguments().contains("--check-mesh-preview")) {
    ScannerMeshPreview preview;
    preview.resize(480,360);
    preview.setMesh({{{-.3f,-.2f,.8f},{.3f,-.2f,.8f},{0,.25f,.9f}}});
    preview.show();app.processEvents();
    const QImage image=preview.grab().toImage();
    int green=0;
    for(int y=0;y<image.height();++y) for(int x=0;x<image.width();++x) {
      const QColor pixel=image.pixelColor(x,y);
      if(pixel.green()>pixel.red()*1.5 && pixel.green()>pixel.blue()) ++green;
    }
    return green>0?0:2;
  }
  if (app.arguments().contains("--check-color-mesh")) {
    KinectOneRemoldProtocol::ReplyExtensionV2 extension{};
    extension.base.flags=7;extension.base.colorFx=1000;
    extension.base.colorFy=1000;extension.base.colorCx=960;
    extension.base.colorCy=540;extension.base.colorShiftD=1000;
    extension.base.colorShiftM=0;
    extension.colorPolynomial[7]=1;extension.colorPolynomial[18]=1;
    const DepthIntrinsics camera{64,48,70,70,31.5,23.5,
        QByteArray(reinterpret_cast<const char*>(&extension),sizeof(extension))};
    QByteArray depth(64*48*2,0);
    for(int i=0;i<64*48;++i) {
      depth[i*2]=char(1000&255);depth[i*2+1]=char(1000>>8);
    }
    QImage color(1920,1080,QImage::Format_RGB32);color.fill(QColor(220,35,25));
    TsdfVolume volume(64,.02f,.08f);volume.reset({0,0,1});
    for(int i=0;i<4;++i)if(!volume.integrate(depth,camera,nullptr,ScannerPose{},2,&color))return 2;
    const auto mesh=volume.mesh(2);
    int red=0;for(const auto& t:mesh)if(qRed(t.ca)>170&&qGreen(t.ca)<80)++red;
    if(mesh.size()<50||red<10)return 3;
    QTemporaryDir directory;if(!directory.isValid())return 4;
    QString error;const QString path=directory.path()+"/mesh.ply";
    if(!ScannerMeshWriter::write(path,mesh,error))return 5;
    QFile file(path);if(!file.open(QIODevice::ReadOnly))return 6;
    const QByteArray data=file.readAll();const int end=data.indexOf("end_header\n");
    if(!data.contains("property uchar red")||end<0||end+11+15>data.size())return 7;
    const int vertex=end+11;
    if(quint8(data[vertex+12])!=220||quint8(data[vertex+13])!=35||
       quint8(data[vertex+14])!=25)return 8;
    const QString objPath=directory.path()+"/mesh.obj";
    if(!ScannerMeshWriter::write(objPath,mesh,error))return 9;
    QFile obj(objPath);if(!obj.open(QIODevice::ReadOnly))return 10;
    return obj.readLine().contains("0.86275 0.13725 0.09804")?0:11;
  }
#ifdef Q_OS_MACOS
  if (app.arguments().contains("--check-body-model")) {
    OnnxBodyTracker tracker(QCoreApplication::applicationDirPath() +
                            "/../Resources/models/movenet.onnx");
    if (!tracker.ready()) {
      fprintf(stderr, "MoveNet model: %s\n", tracker.error().toUtf8().constData());
      return 2;
    }
    QImage sample(640, 480, QImage::Format_RGB32);
    sample.fill(Qt::black);
    return tracker.annotate(sample).isNull() ? 3 : 0;
  }
#endif
#ifdef REMOLD_HAS_ONNX
  if (app.arguments().contains("--check-face-model")) {
#ifdef Q_OS_MACOS
    const QString path=QCoreApplication::applicationDirPath()+"/../Resources/models/face-landmarks.onnx";
#else
    const QString path=QCoreApplication::applicationDirPath()+"/models/face-landmarks.onnx";
#endif
    OnnxFaceTracker tracker(path);
    if(!tracker.ready()) {
      fprintf(stderr,"Face model: %s\n",tracker.error().toUtf8().constData());return 2;
    }
    QImage sample(640,480,QImage::Format_RGB32);sample.fill(Qt::black);
    std::array<BodyKeypoint,17> body{};
    body[0]={.5f,.4f,.9f};body[3]={.42f,.42f,.9f};body[4]={.58f,.42f,.9f};
    tracker.track(sample,body);
    const QString faceSample=qEnvironmentVariable("REMOLD_FACE_SAMPLE");
    if(!faceSample.isEmpty()) {
      const QImage real(faceSample);
      if(real.isNull())return 4;
      if(qEnvironmentVariableIsSet("REMOLD_FACE_PROFILE")) {
        body[0]={.485f,.50f,.9f};body[3]={};body[4]={.65f,.50f,.9f};
        body[5]={.46f,.72f,.9f};body[6]={.75f,.74f,.9f};
      } else {
        body[0]={.35f,.18f,.9f};body[3]={.30f,.19f,.9f};body[4]={.41f,.19f,.9f};
      }
      const QJsonObject captured=tracker.track(real,body);
      fprintf(stderr,"Face sample: %d landmarks, %d expressions\n",
          int(captured.value("landmarks").toArray().size()),
          int(captured.value("blendshapes").toObject().size()));
      if(captured.value("landmarks").toArray().size()!=478||
         captured.value("blendshapes").toObject().size()!=51)return 5;
    }
    return tracker.error().isEmpty()?0:3;
  }
  if (app.arguments().contains("--check-hand-model")) {
#ifdef Q_OS_MACOS
    const QString path = QCoreApplication::applicationDirPath() + "/../Resources/models/rtmpose-hand.onnx";
#else
    const QString path = QCoreApplication::applicationDirPath() + "/models/rtmpose-hand.onnx";
#endif
    OnnxHandTracker tracker(qEnvironmentVariable("REMOLD_HAND_MODEL", path));
    if (!tracker.ready()) {
      fprintf(stderr, "RTMPose Hand model: %s\n", tracker.error().toUtf8().constData());
      return 2;
    }
    QImage sample(640, 480, QImage::Format_RGB32); sample.fill(Qt::black);
    std::array<BodyKeypoint, 17> body{};
    body[9] = {.45f, .50f, .9f}; body[7] = {.45f, .30f, .9f};
    return tracker.annotate(sample, body).isNull() || !tracker.error().isEmpty() ? 3 : 0;
  }
#endif
  MainWindow window;
  if (app.arguments().contains("--check-audio")) {
    MicrophoneWindow microphones;
    return microphones.checkLiveAudio();
  }
  if (app.arguments().contains("--check-acoustic")) {
    AcousticWindow acoustic;
    return acoustic.checkLiveScan();
  }
  if (app.arguments().contains("--check-surveillance")) {
    SurveillanceWindow surveillance;
    return surveillance.checkLiveCameras();
  }
  if (app.arguments().contains("--check-surveillance-recording")) {
    SurveillanceWindow surveillance;
    return surveillance.checkRecording();
  }
  if (app.arguments().contains("--check-live-preview")) return window.checkLiveStreams();
  if (app.arguments().contains("--check-live-body-hands")) return window.checkLiveBodyHands();
  if (app.arguments().contains("--check-color-calibration")) return window.checkColorCalibration();
  if (app.arguments().contains("--check-live-fusion")) {
    ScannerWindow scanner;
    return scanner.checkLiveFusion();
  }
  if (app.arguments().contains("--check-live-fusion-long")) {
    ScannerWindow scanner;
    return scanner.checkLiveFusion(30);
  }
  if (app.arguments().contains("--check-live-turntable")) {
    ScannerWindow scanner;
    return scanner.checkLiveFusion(10,true);
  }
#ifdef REMOLD_HAS_ONNX
  if (app.arguments().contains("--check-live-mocap")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive();
  }
  if (app.arguments().contains("--check-live-face")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive(true,true);
  }
  if (app.arguments().contains("--check-live-face-only")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive(true,false);
  }
  if (app.arguments().contains("--check-live-face-dedicated")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive(true,false,false,true);
  }
  if (app.arguments().contains("--check-live-body-only")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive(false,true,true,false);
  }
  if (app.arguments().contains("--check-live-compact")) {
    MotionCaptureWindow mocap;
    return mocap.checkLive(false,true,true,false);
  }
#endif
  window.show();
  return app.exec();
}

#include "main.moc"
