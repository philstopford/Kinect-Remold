#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace remold::macusb {

struct Device {
  std::uint64_t registryId = 0;
  std::uint16_t vendorId = 0;
  std::uint16_t productId = 0;
  std::uint32_t locationId = 0;
  std::uint8_t speed = 0;
  std::string serial;
};

struct IsoPacket {
  std::uint32_t offset = 0;
  std::uint32_t actualLength = 0;
  int status = 0;
};

std::vector<Device> enumerate(std::uint16_t vendorId);

class Session {
 public:
  Session();
  ~Session();
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  bool open(const Device& device, std::uint8_t interfaceNumber);
  bool attachInterface(const Device& device, std::uint8_t interfaceNumber);
  void close();
  bool resetDevice();
  bool selectAlternate(std::uint8_t interfaceNumber, std::uint8_t alternate);
  bool control(std::uint8_t requestType, std::uint8_t request, std::uint16_t value,
               std::uint16_t index, std::vector<std::uint8_t>& data, int timeoutMs);
  bool bulk(std::uint8_t endpoint, std::vector<std::uint8_t>& data,
            std::size_t& transferred, int timeoutMs);
  bool iso(std::uint8_t endpoint, std::uint32_t packetBytes,
           std::vector<std::uint8_t>& data, std::vector<IsoPacket>& packets);
  std::uint32_t maxIsoPacketBytes(std::uint8_t endpoint);

 private:
  friend class IsoStream;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class IsoStream {
 public:
  IsoStream();
  ~IsoStream();
  IsoStream(const IsoStream&) = delete;
  IsoStream& operator=(const IsoStream&) = delete;
  bool start(Session& session, std::uint8_t endpoint, std::uint32_t packetBytes);
  bool next(std::vector<std::uint8_t>& data, std::vector<IsoPacket>& packets, int timeoutMs);
  void stop();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
