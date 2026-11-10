#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>
#include "NativeUsbHost.hpp"
#include "../modules/kinect-one-remold/shared/KinectOneNativeUsb.h"
#include "../modules/kinect-one-remold/shared/KinectOneDepthEngine.h"
#include "../modules/kinect-one-remold/shared/KinectOneRemoldProtocol.h"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
std::atomic<bool> running{true};
std::mutex sessionMapMutex;
std::unordered_map<std::string, std::shared_ptr<std::mutex>> sessionMutexes;
std::shared_ptr<std::mutex> sessionMutex(const std::string& id) {
  std::lock_guard<std::mutex> lock(sessionMapMutex);
  auto& value = sessionMutexes[id];
  if (!value) value = std::make_shared<std::mutex>();
  return value;
}
void stop(int) { running = false; }

std::string clean(std::string value) {
  for (char& ch : value) if (ch == '\t' || ch == '\r' || ch == '\n') ch = '_';
  return value;
}

std::string deviceId(const remold::macusb::Device& device) {
  std::string serial;
  for (const unsigned char ch : device.serial) {
    if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
      serial.push_back(static_cast<char>(ch));
    if (serial.size() == 48) break;
  }
  char id[40];
  if (!serial.empty()) {
    std::snprintf(id, sizeof(id), "mac-%04x-", device.productId);
    return std::string(id) + serial;
  }
  if (device.locationId) std::snprintf(id, sizeof(id), "mac-%04x-%08x", device.productId, device.locationId);
  else std::snprintf(id, sizeof(id), "mac-%016llx", static_cast<unsigned long long>(device.registryId));
  return id;
}

bool writeAll(int fd, const void* input, std::size_t bytes) {
  const auto* cursor = static_cast<const std::uint8_t*>(input);
  while (bytes) {
    const ssize_t count = send(fd, cursor, bytes, 0);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    cursor += count; bytes -= static_cast<std::size_t>(count);
  }
  return true;
}

bool readAll(int fd, void* output, std::size_t bytes) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  while (bytes) {
    const ssize_t count = recv(fd, cursor, bytes, 0);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    cursor += count; bytes -= static_cast<std::size_t>(count);
  }
  return true;
}

bool sendFrame(int fd, KinectOneRemoldProtocol::StreamMode mode, std::uint32_t width,
               std::uint32_t height, KinectOneRemoldProtocol::PixelFormat format,
               const std::uint8_t* bytes, std::size_t length, std::uint64_t number) {
  KinectOneRemoldProtocol::FrameHeader header{};
  header.mode = mode; header.width = width; header.height = height;
  header.pixelFormat = format; header.payloadBytes = static_cast<std::uint32_t>(length);
  header.frameNumber = number;
  header.tickMs = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  return writeAll(fd, &header, sizeof(header)) && writeAll(fd, bytes, length);
}

std::vector<std::string> discover(bool one, const std::string& sdk) {
  std::vector<std::string> rows;
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    const int product = device.productId;
    const bool oneProduct = product == 0x02c4 || product == 0x02d8;
    const bool firstProduct = product == 0x02ae || product == 0x02bf;
    if (one ? oneProduct : firstProduct) {
      const std::string id = deviceId(device);
      const char* generation = one ? "xbox-one" : "xbox-360";
      const char* module = one ? "kinect-one-remold" : "kinect-xbox-360-remold";
      const std::string label = one ? (product == 0x02c4 ? "Kinect One 1520" : "Kinect One 1565") :
          product == 0x02bf ? "Kinect for Windows 1517" : "Kinect Xbox 360";
      const bool ready = one && device.speed >= 4;
      const char* state = one && device.speed != 0 && device.speed < 4 ? "USB3Required" :
                          ready ? "Ready" : "NativeTransportPending";
      const std::string camera = ready ? sdk.substr(0, sdk.find_last_of('/')) + "/camera.sock" : "";
      std::string row = id + '\t' + clean(label) + '\t' + state + "\t\t" + camera + "\t\t\t\t" +
          sdk + '\t' + module + '\t' + generation +
          (ready ? "\tcolor,depth,infrared,native-metric-depth,native-jpeg-color,native-iousbhost" :
                   "\tnative-usb-discovery") +
          (ready ? "\t1920\t1080\t512\t424\t512\t424\t30\t30\n" :
                   "\t0\t0\t0\t0\t0\t0\t0\t0\n");
      rows.push_back(std::move(row));
    }
  }
  return rows;
}

std::string probe(const std::string& id, bool one) {
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    const bool matchingProduct = one ? (device.productId == 0x02c4 || device.productId == 0x02d8)
                                     : (device.productId == 0x02ae || device.productId == 0x02bf);
    if (!matchingProduct || deviceId(device) != id) continue;
    if (one && device.speed != 0 && device.speed < 4) return "ERR USB3_REQUIRED\n";
    remold::macusb::Session session;
    if (!session.open(device, 0)) return "ERR IOUSBHOST_INTERFACE_OPEN\n";
    if (one && !session.attachInterface(device, 1)) return "ERR IOUSBHOST_DEPTH_INTERFACE_OPEN\n";
    return one ? "OK IOUSBHOST_CONTROL_AND_DEPTH_INTERFACES_OPEN\n" : "OK IOUSBHOST_INTERFACE_OPEN\n";
  }
  return "ERR NOT_FOUND\n";
}

std::string info(const std::string& id) {
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    if ((device.productId != 0x02c4 && device.productId != 0x02d8) || deviceId(device) != id)
      continue;
    if (device.speed != 0 && device.speed < 4) return "ERR USB3_REQUIRED\n";
    remold::macusb::Session session;
    if (!session.open(device, KinectOneNativeUsb::ControlInterface)) return "ERR IOUSBHOST_INTERFACE_OPEN\n";
    constexpr std::uint32_t sequence = 1;
    const auto request = KinectOneNativeUsb::encodeCommand(sequence, KinectOneNativeUsb::command::firmwareVersions());
    std::vector<std::uint8_t> output(request.begin(), request.end());
    std::size_t written = 0;
    if (!session.bulk(KinectOneNativeUsb::CommandOut, output, written, 1500) || written != output.size())
      return "ERR COMMAND_WRITE\n";
    std::vector<std::uint8_t> response(0x200);
    std::size_t received = 0;
    if (!session.bulk(KinectOneNativeUsb::CommandIn, response, received, 1500)) return "ERR COMMAND_READ\n";
    std::vector<std::uint8_t> completion(KinectOneNativeUsb::CompletionBytes);
    std::size_t completeBytes = 0;
    if (!session.bulk(KinectOneNativeUsb::CommandIn, completion, completeBytes, 1500) ||
        !KinectOneNativeUsb::validCompletion(completion.data(), completeBytes, sequence))
      return "ERR COMMAND_COMPLETION\n";
    return "OK NATIVE_BULK bytes=" + std::to_string(received) + "\n";
  }
  return "ERR NOT_FOUND\n";
}

std::string resetDevice(const std::string& id) {
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    if (!KinectOneNativeUsb::supportedProduct(device.productId) || deviceId(device) != id) continue;
    remold::macusb::Session session;
    if (!session.open(device, KinectOneNativeUsb::ControlInterface)) return "ERR INTERFACE_OPEN\n";
    return session.resetDevice() ? "OK DEVICE_RESET\n" : "ERR DEVICE_RESET\n";
  }
  return "ERR NOT_FOUND\n";
}

bool nativeCommand(remold::macusb::Session& session, std::uint32_t sequence,
                   const KinectOneNativeUsb::CommandSpec& spec,
                   std::vector<std::uint8_t>& response, int timeoutMs) {
  auto output = KinectOneNativeUsb::encodeCommand(sequence, spec);
  std::size_t written = 0;
  if (!session.bulk(KinectOneNativeUsb::CommandOut, output, written, timeoutMs) || written != output.size())
    return false;
  response.clear();
  if (spec.maxResponseBytes) {
    response.resize(spec.maxResponseBytes);
    std::size_t received = 0;
    if (!session.bulk(KinectOneNativeUsb::CommandIn, response, received, timeoutMs)) return false;
  }
  std::vector<std::uint8_t> completion(KinectOneNativeUsb::CompletionBytes);
  std::size_t completeBytes = 0;
  return session.bulk(KinectOneNativeUsb::CommandIn, completion, completeBytes, timeoutMs) &&
         KinectOneNativeUsb::validCompletion(completion.data(), completeBytes, sequence);
}

std::string startTest(const std::string& id, int streamFd = -1, std::uint32_t streamMask = 0,
                      std::uint8_t extensionRequest = 0) {
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    if ((device.productId != 0x02c4 && device.productId != 0x02d8) || deviceId(device) != id)
      continue;
    if (device.speed != 0 && device.speed < 4) return "ERR USB3_REQUIRED\n";
    remold::macusb::Session session;
    if (!session.open(device, 0) || !session.attachInterface(device, 1)) return "ERR INTERFACE_OPEN\n";
    std::vector<std::uint8_t> empty;
    auto video = [&](bool enabled) {
      const std::uint16_t index = static_cast<std::uint16_t>(((enabled ? 0u : 3u) << 8) | 0u);
      return session.control(0x01, 0x03, 0, index, empty, 1000);
    };
    std::uint32_t sequence = 1;
    std::vector<std::uint8_t> response;
    std::vector<std::uint8_t> depth, phase, color;
    auto decoder = std::make_unique<KinectOneDepthEngine::Decoder>();
    auto command = [&](const KinectOneNativeUsb::CommandSpec& spec, int timeout = 1500) {
      return nativeCommand(session, sequence++, spec, response, timeout);
    };
    bool enabled = false;
    std::string stage = "";
    if (!session.selectAlternate(1, 0)) stage = "DEPTH_ALT_ZERO";
    else if (!session.control(0x00, 0x31, 40, 0, empty, 1000)) stage = "ISO_DELAY";
    else if (!session.control(0x00, 0x03, 48, 0, empty, 1000) ||
             !session.control(0x00, 0x03, 49, 0, empty, 1000)) stage = "USB3_POWER_STATE";
    else if (!video(false)) stage = "VIDEO_SUSPEND";
    else if (!video(true)) stage = "VIDEO_ENABLE";
    if (stage.empty()) {
      if (!command(KinectOneNativeUsb::command::firmwareVersions()) ||
          !command(KinectOneNativeUsb::command::hardwareInfo()) ||
          !command(KinectOneNativeUsb::command::serialNumber())) stage = "IDENTITY";
      else if (!command(KinectOneNativeUsb::command::depthParameters(), 4000)) stage = "DEPTH_CALIBRATION";
      else { depth = response;
        if (!command(KinectOneNativeUsb::command::p0Tables(), 5000)) stage = "PHASE_CALIBRATION";
        else { phase = response;
          if (!command(KinectOneNativeUsb::command::colorParameters(), 4000)) stage = "COLOR_CALIBRATION";
          else color = response;
        }
      }
      if (stage.empty()) {
        if (!decoder->configure(depth, phase) || color.size() < 64) stage = "CALIBRATION_DECODE";
      }
    }
    if (stage.empty() && (!command(KinectOneNativeUsb::command::mode(true, 0x00640064u)) ||
                          !command(KinectOneNativeUsb::command::mode(false)))) stage = "MODE_INIT";
    if (stage.empty()) {
      bool ready = false;
      for (int attempt = 0; attempt < 50; ++attempt) {
        if (!command(KinectOneNativeUsb::command::status())) { stage = "STATUS_QUERY"; break; }
        std::uint32_t status = 0;
        if (response.size() >= 4) std::memcpy(&status, response.data(), 4);
        if (status & 1u) { ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (stage.empty() && !ready) stage = "SENSOR_NOT_READY";
    }
    if (stage.empty() && !command(KinectOneNativeUsb::command::initStreams())) stage = "STREAM_INIT";
    if (stage.empty() && !session.selectAlternate(1, 1)) stage = "DEPTH_ALT_ONE";
    if (stage.empty() && !command(KinectOneNativeUsb::command::status())) stage = "POST_ALT_STATUS";
    if (stage.empty()) {
      if (!command(KinectOneNativeUsb::command::streaming(true))) stage = "STREAM_ENABLE";
      else enabled = true;
    }
    std::uint32_t isoBytes = 0;
    std::size_t activePackets = 0;
    int lastPacketStatus = 0;
    remold::macusb::IsoStream isoStream;
    if (enabled) {
      isoBytes = session.maxIsoPacketBytes(KinectOneNativeUsb::DepthIn);
      if (isoBytes == 0) stage = "ISO_PACKET_SIZE";
      else {
        std::vector<std::uint8_t> data;
        std::vector<remold::macusb::IsoPacket> packets;
        if (!isoStream.start(session, KinectOneNativeUsb::DepthIn, isoBytes)) stage = "ISO_QUEUE_START";
        for (int attempt = 0; stage.empty() && attempt < 100 && activePackets == 0; ++attempt) {
          if (!isoStream.next(data, packets, 1000)) { stage = "ISO_READ"; break; }
          for (const auto& packet : packets) {
            lastPacketStatus = packet.status;
            if (packet.status == 0 && packet.actualLength) ++activePackets;
          }
        }
      }
    }
    if (streamFd >= 0 && stage.empty()) {
      KinectOneRemoldProtocol::Reply reply{};
      reply.result = KinectOneRemoldProtocol::ResultOk;
      reply.acceptedMask = streamMask;
      reply.width = 512; reply.height = 424;
      reply.capabilities = KinectOneRemoldProtocol::CapabilityRgbDepthConcurrent |
                           KinectOneRemoldProtocol::CapabilityInfrared |
                           KinectOneRemoldProtocol::CapabilityNativeMetricDepth |
                           KinectOneRemoldProtocol::CapabilityNativeJpegColor;
      reply.maxPayloadBytes = 16u * 1024u * 1024u;
      const auto& calibration = decoder->calibration();
      reply.depthCalibrationValid = extensionRequest == KinectOneRemoldProtocol::kReplyExtensionFullRequest
          ? KinectOneRemoldProtocol::kDepthCalibrationNativeFull
          : extensionRequest == KinectOneRemoldProtocol::kReplyExtensionRequest
              ? KinectOneRemoldProtocol::kDepthCalibrationNativeExtended
              : KinectOneRemoldProtocol::kDepthCalibrationNative;
      reply.depthConstShift = calibration.fx; reply.depthEmitterDistance = calibration.fy;
      reply.depthReferenceDistance = calibration.cx; reply.depthReferencePixelSize = calibration.cy;
      const auto extension = KinectOneRemoldProtocol::makeReplyExtension(
          calibration.k1, calibration.k2, calibration.k3, calibration.p1, calibration.p2,
          color.data(), color.size());
      const auto fullExtension = KinectOneRemoldProtocol::makeReplyExtensionV2(
          calibration.k1, calibration.k2, calibration.k3, calibration.p1, calibration.p2,
          color.data(), color.size());
      const bool replyWritten = writeAll(streamFd, &reply, sizeof(reply));
      const bool extensionWritten = !replyWritten ? false :
          extensionRequest == KinectOneRemoldProtocol::kReplyExtensionFullRequest
          ? writeAll(streamFd, &fullExtension, sizeof(fullExtension))
          : extensionRequest == KinectOneRemoldProtocol::kReplyExtensionRequest
              ? writeAll(streamFd, &extension, sizeof(extension)) : true;
      if (replyWritten && extensionWritten) {
        auto assembler = std::make_unique<KinectOneNativeUsb::DepthFrameAssembler>();
        std::vector<std::uint8_t> isoData;
        std::vector<remold::macusb::IsoPacket> packets;
        std::atomic<std::uint64_t> frameNumber{0};
        std::atomic<bool> colorRunning{true}, outputOk{true};
        std::mutex writeMutex;
        std::mutex rawMutex;
        std::condition_variable rawReady;
        std::deque<std::vector<std::uint8_t>> rawFrames;
        std::atomic<bool> decodeRunning{true};
        std::thread decodeThread([&] {
          while (running && outputOk) {
            std::vector<std::uint8_t> raw;
            {
              std::unique_lock<std::mutex> lock(rawMutex);
              rawReady.wait(lock, [&] { return !rawFrames.empty() || !decodeRunning; });
              if (!decodeRunning && rawFrames.empty()) break;
              raw = std::move(rawFrames.front()); rawFrames.pop_front();
            }
            std::vector<std::uint16_t> depthFrame, infrared;
            const bool needIr = (streamMask & KinectOneRemoldProtocol::StreamInfrared) != 0;
            if (!decoder->decode(raw, depthFrame, needIr ? &infrared : nullptr)) continue;
            if (streamMask & KinectOneRemoldProtocol::StreamDepth) {
              std::lock_guard<std::mutex> lock(writeMutex);
              if (!sendFrame(streamFd, KinectOneRemoldProtocol::StreamMode::Depth, 512, 424,
                             KinectOneRemoldProtocol::DepthMm16,
                             reinterpret_cast<const std::uint8_t*>(depthFrame.data()), depthFrame.size()*2,
                             ++frameNumber)) outputOk = false;
            }
            if (needIr && outputOk) {
              std::lock_guard<std::mutex> lock(writeMutex);
              if (!sendFrame(streamFd, KinectOneRemoldProtocol::StreamMode::Infrared, 512, 424,
                             KinectOneRemoldProtocol::InfraredU16,
                             reinterpret_cast<const std::uint8_t*>(infrared.data()), infrared.size()*2,
                             ++frameNumber)) outputOk = false;
            }
          }
        });
        std::thread colorThread;
        if (streamMask & (KinectOneRemoldProtocol::StreamRgb | KinectOneRemoldProtocol::StreamRgbHighQuality)) {
          colorThread = std::thread([&] {
            KinectOneNativeUsb::JpegAssembler jpeg;
            std::vector<std::uint8_t> colorChunk, jpegFrame;
            while (running && colorRunning && outputOk) {
              colorChunk.resize(1024 * 1024);
              std::size_t transferred = 0;
              if (!session.bulk(KinectOneNativeUsb::ColorIn, colorChunk, transferred, 100) || !transferred) continue;
              jpeg.push(colorChunk.data(), transferred);
              while (jpeg.pop(jpegFrame)) {
                const auto mode = (streamMask & KinectOneRemoldProtocol::StreamRgbHighQuality)
                    ? KinectOneRemoldProtocol::StreamMode::RgbHighQuality : KinectOneRemoldProtocol::StreamMode::Rgb;
                std::lock_guard<std::mutex> lock(writeMutex);
                if (!sendFrame(streamFd, mode, 1920, 1080, KinectOneRemoldProtocol::Jpeg,
                               jpegFrame.data(), jpegFrame.size(), ++frameNumber)) { outputOk = false; break; }
              }
            }
          });
        }
        std::uint64_t bursts = 0;
        while (running && outputOk) {
          pollfd peer{streamFd, POLLHUP | POLLERR, 0};
          if (poll(&peer, 1, 0) > 0 && peer.revents) { std::fprintf(stderr, "native camera peer closed\n"); break; }
          if (!isoStream.next(isoData, packets, 1000)) {
            std::fprintf(stderr, "native camera isochronous read failed after %llu bursts\n", static_cast<unsigned long long>(bursts)); break;
          }
          ++bursts;
          for (const auto& packet : packets) {
            if (packet.status != 0 || !packet.actualLength ||
                static_cast<std::size_t>(packet.offset) + packet.actualLength > isoData.size()) continue;
            std::vector<std::uint8_t> raw;
            std::uint32_t timestamp = 0;
            if (!assembler->pushIsoPacket(isoData.data() + packet.offset, packet.actualLength, raw, timestamp)) continue;
            {
              std::lock_guard<std::mutex> lock(rawMutex);
              if (rawFrames.size() == 3) rawFrames.pop_front();
              rawFrames.push_back(std::move(raw));
            }
            rawReady.notify_one();
          }
          if (!outputOk) break;
        }
        colorRunning = false;
        decodeRunning = false;
        rawReady.notify_all();
        if (colorThread.joinable()) colorThread.join();
        decodeThread.join();
      }
    }
    isoStream.stop();
    if (enabled) {
      session.selectAlternate(1, 0);
      command(KinectOneNativeUsb::command::mode(true, 0x00640064u));
      command(KinectOneNativeUsb::command::mode(false));
      command(KinectOneNativeUsb::command::stop());
      command(KinectOneNativeUsb::command::streaming(false));
      command(KinectOneNativeUsb::command::mode(true));
      command(KinectOneNativeUsb::command::mode(false));
      command(KinectOneNativeUsb::command::mode(true));
      command(KinectOneNativeUsb::command::mode(false));
    }
    video(false);
    return stage.empty() ? "OK NATIVE_STREAM_START_STOP iso_packet=" + std::to_string(isoBytes) +
                            " active_packets=" + std::to_string(activePackets) +
                            " last_status=" + std::to_string(lastPacketStatus) + "\n" : "ERR " + stage + "\n";
  }
  return "ERR NOT_FOUND\n";
}

std::string calibration(const std::string& id) {
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    if ((device.productId != 0x02c4 && device.productId != 0x02d8) || deviceId(device) != id)
      continue;
    if (device.speed != 0 && device.speed < 4) return "ERR USB3_REQUIRED\n";
    remold::macusb::Session session;
    if (!session.open(device, KinectOneNativeUsb::ControlInterface)) return "ERR IOUSBHOST_INTERFACE_OPEN\n";
    std::vector<std::uint8_t> response;
    if (!nativeCommand(session, 1, KinectOneNativeUsb::command::depthParameters(), response, 4000))
      return "ERR DEPTH_PARAMETERS\n";
    const std::size_t depthBytes = response.size();
    if (!nativeCommand(session, 2, KinectOneNativeUsb::command::p0Tables(), response, 5000))
      return "ERR PHASE_TABLES\n";
    const std::size_t phaseBytes = response.size();
    if (!nativeCommand(session, 3, KinectOneNativeUsb::command::colorParameters(), response, 4000))
      return "ERR COLOR_PARAMETERS\n";
    return "OK NATIVE_CALIBRATION depth=" + std::to_string(depthBytes) +
           " phase=" + std::to_string(phaseBytes) + " color=" + std::to_string(response.size()) + "\n";
  }
  return "ERR NOT_FOUND\n";
}

int server(const std::string& path) {
  if (path.size() >= sizeof(sockaddr_un::sun_path)) return -1;
  struct stat existing{};
  if (lstat(path.c_str(), &existing) == 0) {
    if (!S_ISSOCK(existing.st_mode)) { errno = EEXIST; return -1; }
    int probe = socket(AF_UNIX, SOCK_STREAM, 0);
    if (probe >= 0) {
      sockaddr_un address{}; address.sun_family = AF_UNIX;
      std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
      const bool active = connect(probe, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0;
      close(probe);
      if (active) { errno = EADDRINUSE; return -1; }
    }
    unlink(path.c_str());
  }
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  sockaddr_un address{}; address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      chmod(path.c_str(), 0600) != 0 || listen(fd, 16) != 0) {
    const int failure = errno;
    std::fprintf(stderr, "native USB socket %s: %s\n", path.c_str(), std::strerror(failure));
    close(fd); unlink(path.c_str()); errno = failure; return -1;
  }
  return fd;
}

void client(int fd, bool one, const std::string& sdk) {
  char request[256]{};
  const ssize_t size = read(fd, request, sizeof(request)-1);
  if (size <= 0) return;
  std::string command(request, size);
  while (!command.empty() && (command.back() == '\n' || command.back() == '\r')) command.pop_back();
  std::string response;
  if (command == "LIST") {
    for (const auto& row : discover(one, sdk)) response += row;
  } else if (command.rfind("GET ", 0) == 0) {
    const std::string id = command.substr(4);
    response = "ERR NOT_FOUND\n";
    for (const auto& row : discover(one, sdk))
      if (row.rfind(id + '\t', 0) == 0) { response = row; break; }
  } else if (command.rfind("CONTROL\t", 0) == 0) {
    const auto secondTab = command.find('\t', 8);
    if (secondTab == std::string::npos) response = "ERR BAD_COMMAND\n";
    else {
      const std::string id = command.substr(8, secondTab - 8);
      const std::string action = command.substr(secondTab + 1);
      response = "ERR NOT_FOUND\n";
      for (const auto& row : discover(one, sdk)) {
        if (row.rfind(id + '\t', 0) != 0) continue;
        const auto stateStart = row.find('\t', id.size() + 1);
        const auto stateEnd = stateStart == std::string::npos ? stateStart : row.find('\t', stateStart + 1);
        const std::string state = stateEnd == std::string::npos ? "" : row.substr(stateStart + 1, stateEnd - stateStart - 1);
        if (action == "Status") response = "OK " + state + "\n";
        else if (action == "OpenCamera" || action == "OneColor" || action == "OneDepth" || action == "OneInfrared")
          response = state == "Ready" ? "OK Ready\n" : "ERR NOT_READY\n";
        else response = "ERR UNSUPPORTED_ACTION\n";
        break;
      }
    }
  } else if (command.rfind("PROBE ", 0) == 0) {
    std::string id = command.substr(6);
    while (!id.empty() && (id.back() == '\n' || id.back() == '\r')) id.pop_back();
    if (one) {
      std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
      response = lock.owns_lock() ? probe(id, one) : "ERR DEVICE_BUSY\n";
    } else response = probe(id, one);
  } else if (one && command.rfind("INFO ", 0) == 0) {
    std::string id = command.substr(5);
    while (!id.empty() && (id.back() == '\n' || id.back() == '\r')) id.pop_back();
    std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
    response = lock.owns_lock() ? info(id) : "ERR DEVICE_BUSY\n";
  } else if (one && command.rfind("RESET ", 0) == 0) {
    std::string id = command.substr(6);
    while (!id.empty() && (id.back() == '\n' || id.back() == '\r')) id.pop_back();
    std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
    response = lock.owns_lock() ? resetDevice(id) : "ERR DEVICE_BUSY\n";
  } else if (one && command.rfind("CALIBRATION ", 0) == 0) {
    std::string id = command.substr(12);
    while (!id.empty() && (id.back() == '\n' || id.back() == '\r')) id.pop_back();
    std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
    response = lock.owns_lock() ? calibration(id) : "ERR DEVICE_BUSY\n";
  } else if (one && command.rfind("STARTTEST ", 0) == 0) {
    std::string id = command.substr(10);
    while (!id.empty() && (id.back() == '\n' || id.back() == '\r')) id.pop_back();
    std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
    response = lock.owns_lock() ? startTest(id) : "ERR DEVICE_BUSY\n";
  } else response = "ERR UNSUPPORTED_NATIVE_TRANSPORT\n";
  const char* data = response.data(); size_t left = response.size();
  while (left) {
    const ssize_t written = write(fd, data, left);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) break;
    data += written; left -= static_cast<size_t>(written);
  }
}

void cameraClient(int fd) {
  KinectOneRemoldProtocol::RequestV2 request{};
  if (!readAll(fd, &request.request, sizeof(request.request))) return;
  // The wire format permits a legacy 16-byte request and a device-scoped
  // 80-byte request. Once any device-ID byte arrives, require the entire ID:
  // a fragmented write must never silently select the first sensor.
  pollfd more{fd, POLLIN, 0};
  const int pending = poll(&more, 1, 1000);
  if (pending < 0 && errno == EINTR) return;
  if (pending > 0 && (more.revents & POLLIN) &&
      !readAll(fd, request.deviceId, sizeof(request.deviceId))) return;
  KinectOneRemoldProtocol::Reply reply{};
  const std::uint32_t mask = request.request.streamMask;
  if (request.request.magic != KinectOneRemoldProtocol::kMagic ||
      request.request.version != KinectOneRemoldProtocol::kVersion ||
      request.request.command != KinectOneRemoldProtocol::Command::SubscribeStreams ||
      mask == 0 || (mask & ~15u)) {
    reply.result = KinectOneRemoldProtocol::ResultUnsupported;
    writeAll(fd, &reply, sizeof(reply)); return;
  }
  std::string id(request.deviceId, strnlen(request.deviceId, sizeof(request.deviceId)));
  if (id.empty()) {
    for (const auto& device : remold::macusb::enumerate(0x045e))
      if (KinectOneNativeUsb::supportedProduct(device.productId) && device.speed >= 4) {
        id = deviceId(device); break;
      }
  }
  if (id.empty()) {
    reply.result = KinectOneRemoldProtocol::ResultDeviceNotFound;
    writeAll(fd, &reply, sizeof(reply)); return;
  }
  bool found = false;
  bool superspeed = false;
  for (const auto& device : remold::macusb::enumerate(0x045e)) {
    if (!KinectOneNativeUsb::supportedProduct(device.productId) || deviceId(device) != id) continue;
    found = true;
    superspeed = device.speed >= 4;
    break;
  }
  if (!found || !superspeed) {
    reply.result = found ? KinectOneRemoldProtocol::ResultStreamingUnavailable
                         : KinectOneRemoldProtocol::ResultDeviceNotFound;
    writeAll(fd, &reply, sizeof(reply)); return;
  }
  std::unique_lock<std::mutex> lock(*sessionMutex(id), std::try_to_lock);
  if (!lock.owns_lock()) {
    reply.result = KinectOneRemoldProtocol::ResultDeviceBusy;
    writeAll(fd, &reply, sizeof(reply)); return;
  }
  const std::uint8_t extensionMarker = KinectOneRemoldProtocol::fullExtensionRequested(request)
      ? KinectOneRemoldProtocol::kReplyExtensionFullRequest
      : KinectOneRemoldProtocol::extensionRequested(request)
          ? KinectOneRemoldProtocol::kReplyExtensionRequest : 0;
  std::string result = startTest(id, fd, mask, extensionMarker);
  if (result == "ERR IDENTITY\n" || result == "ERR STATUS_QUERY\n" ||
      result == "ERR SENSOR_NOT_READY\n") {
    std::fprintf(stderr, "native camera %s: recovering after %s", id.c_str(), result.c_str());
    if (resetDevice(id) == "OK DEVICE_RESET\n") {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      result = startTest(id, fd, mask, extensionMarker);
    }
  }
  if (result.rfind("ERR", 0) == 0) {
    std::fprintf(stderr, "native camera %s: %s", id.c_str(), result.c_str());
    reply.result = KinectOneRemoldProtocol::ResultStreamingUnavailable;
    writeAll(fd, &reply, sizeof(reply));
  }
}

void cameraLoop(int serverFd) {
  std::mutex workerMutex;
  std::condition_variable workerDone;
  unsigned active = 0;
  constexpr unsigned maxClients = 16;
  while (running) {
    pollfd ready{serverFd, POLLIN, 0};
    if (poll(&ready, 1, 250) <= 0 || !(ready.revents & POLLIN)) continue;
    int fd = accept(serverFd, nullptr, nullptr);
    if (fd < 0) continue;
    {
      std::lock_guard<std::mutex> state(workerMutex);
      if (active < maxClients) { ++active; }
      else {
        KinectOneRemoldProtocol::Reply reply{};
        reply.result = KinectOneRemoldProtocol::ResultDeviceBusy;
        writeAll(fd, &reply, sizeof(reply));
        close(fd);
        continue;
      }
    }
    timeval requestTimeout{5, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &requestTimeout, sizeof(requestTimeout));
    timeval sendTimeout{2, 0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));
    std::thread([fd, &active, &workerMutex, &workerDone] {
      cameraClient(fd); close(fd);
      { std::lock_guard<std::mutex> lock(workerMutex); --active; }
      workerDone.notify_all();
    }).detach();
  }
  std::unique_lock<std::mutex> state(workerMutex);
  workerDone.wait(state, [&] { return active == 0; });
}
}

int main() {
  @autoreleasepool {
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop); std::signal(SIGPIPE, SIG_IGN);
    const char* first = std::getenv("REMOLD_MACOS_RUNTIME360");
    const char* second = std::getenv("REMOLD_MACOS_RUNTIME_ONE");
    if (!first || !*first || !second || !*second) {
      std::fprintf(stderr, "Both REMOLD_MACOS_RUNTIME360 and REMOLD_MACOS_RUNTIME_ONE are required.\n");
      return 2;
    }
    const std::string firstPath = std::string(first) + "/sdk.sock";
    const std::string secondPath = std::string(second) + "/sdk.sock";
    const std::string cameraPath = std::string(second) + "/camera.sock";
    int firstFd = server(firstPath), secondFd = server(secondPath), cameraFd = server(cameraPath);
    if (firstFd < 0 || secondFd < 0 || cameraFd < 0) {
      std::perror("native USB SDK socket");
      if (firstFd >= 0) close(firstFd);
      if (secondFd >= 0) close(secondFd);
      if (cameraFd >= 0) close(cameraFd);
      unlink(firstPath.c_str()); unlink(secondPath.c_str()); unlink(cameraPath.c_str()); return 3;
    }
    std::thread cameraThread(cameraLoop, cameraFd);
    while (running) {
      pollfd fds[2]{{firstFd, POLLIN, 0}, {secondFd, POLLIN, 0}};
      if (poll(fds, 2, 250) <= 0) continue;
      for (int i = 0; i < 2; ++i) if (fds[i].revents & POLLIN) {
        int connection = accept(fds[i].fd, nullptr, nullptr);
        if (connection >= 0) { client(connection, i == 1, i == 1 ? secondPath : firstPath); close(connection); }
      }
    }
    cameraThread.join();
    close(firstFd); close(secondFd); close(cameraFd);
    unlink(firstPath.c_str()); unlink(secondPath.c_str()); unlink(cameraPath.c_str());
    return 0;
  }
}
