#import "NativeUsbHost.hpp"
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>

namespace remold::macusb {
namespace {
int registryNumber(io_registry_entry_t entry, CFStringRef key) {
  CFTypeRef value = IORegistryEntryCreateCFProperty(entry, key, kCFAllocatorDefault, 0);
  int number = 0;
  if (value && CFGetTypeID(value) == CFNumberGetTypeID())
    CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberIntType, &number);
  if (value) CFRelease(value);
  return number;
}

io_service_t findDevice(std::uint64_t registryId) {
  io_iterator_t iterator = IO_OBJECT_NULL;
  if (IOServiceGetMatchingServices(kIOMainPortDefault,
      IOServiceMatching("IOUSBHostDevice"), &iterator) != KERN_SUCCESS) return IO_OBJECT_NULL;
  io_service_t result = IO_OBJECT_NULL, entry;
  while ((entry = IOIteratorNext(iterator)) != IO_OBJECT_NULL) {
    std::uint64_t id = 0;
    IORegistryEntryGetRegistryEntryID(entry, &id);
    if (id == registryId) { result = entry; break; }
    IOObjectRelease(entry);
  }
  IOObjectRelease(iterator);
  return result;
}

io_service_t findInterface(const Device& device, std::uint8_t number) {
  io_iterator_t iterator = IO_OBJECT_NULL;
  if (IOServiceGetMatchingServices(kIOMainPortDefault,
      IOServiceMatching("IOUSBHostInterface"), &iterator) != KERN_SUCCESS) return IO_OBJECT_NULL;
  io_service_t result = IO_OBJECT_NULL, entry;
  while ((entry = IOIteratorNext(iterator)) != IO_OBJECT_NULL) {
    if (registryNumber(entry, CFSTR("bInterfaceNumber")) != number) {
      IOObjectRelease(entry); continue;
    }
    bool matches = false;
    io_registry_entry_t current = entry;
    IOObjectRetain(current);
    for (int depth = 0; depth < 8; ++depth) {
      std::uint64_t id = 0;
      IORegistryEntryGetRegistryEntryID(current, &id);
      if (id == device.registryId) { matches = true; break; }
      io_registry_entry_t parent = IO_OBJECT_NULL;
      if (IORegistryEntryGetParentEntry(current, kIOServicePlane, &parent) != KERN_SUCCESS) break;
      IOObjectRelease(current); current = parent;
    }
    IOObjectRelease(current);
    if (!matches) { IOObjectRelease(entry); continue; }
    result = entry; break;
  }
  IOObjectRelease(iterator);
  return result;
}
}

std::vector<Device> enumerate(std::uint16_t vendorId) {
  std::vector<Device> devices;
  io_iterator_t iterator = IO_OBJECT_NULL;
  if (IOServiceGetMatchingServices(kIOMainPortDefault,
      IOServiceMatching("IOUSBHostDevice"), &iterator) != KERN_SUCCESS) return devices;
  io_service_t entry;
  while ((entry = IOIteratorNext(iterator)) != IO_OBJECT_NULL) {
    const int vendor = registryNumber(entry, CFSTR("idVendor"));
    if (vendor == vendorId) {
      Device device{};
      device.vendorId = vendor;
      device.productId = static_cast<std::uint16_t>(registryNumber(entry, CFSTR("idProduct")));
      device.locationId = static_cast<std::uint32_t>(registryNumber(entry, CFSTR("locationID")));
      device.speed = static_cast<std::uint8_t>(registryNumber(entry, CFSTR("USBSpeed")));
      CFTypeRef serial = IORegistryEntryCreateCFProperty(entry, CFSTR("kUSBSerialNumberString"), kCFAllocatorDefault, 0);
      if (serial && CFGetTypeID(serial) == CFStringGetTypeID()) {
        char bytes[128]{};
        if (CFStringGetCString(static_cast<CFStringRef>(serial), bytes, sizeof(bytes), kCFStringEncodingUTF8))
          device.serial = bytes;
      }
      if (serial) CFRelease(serial);
      IORegistryEntryGetRegistryEntryID(entry, &device.registryId);
      devices.push_back(device);
    }
    IOObjectRelease(entry);
  }
  IOObjectRelease(iterator);
  return devices;
}

struct Session::Impl {
  IOUSBHostDevice* device = nil;
  IOUSBHostInterface* interfaces[256]{};
};

Session::Session() : impl_(std::make_unique<Impl>()) {}
Session::~Session() { close(); }

bool Session::open(const Device& device, std::uint8_t interfaceNumber) {
  close();
  @autoreleasepool {
    io_service_t service = findDevice(device.registryId);
    if (service == IO_OBJECT_NULL) return false;
    NSError* error = nil;
    impl_->device = [[IOUSBHostDevice alloc] initWithIOService:service
        options:IOUSBHostObjectInitOptionsNone queue:nil error:&error interestHandler:nil];
    IOObjectRelease(service);
    if (!impl_->device) return false;
    if (!attachInterface(device, interfaceNumber)) { close(); return false; }
    return true;
  }
}

bool Session::attachInterface(const Device& device, std::uint8_t interfaceNumber) {
  if (!impl_->device) return false;
  if (impl_->interfaces[interfaceNumber]) return true;
  @autoreleasepool {
    io_service_t service = findInterface(device, interfaceNumber);
    if (service == IO_OBJECT_NULL) return false;
    NSError* error = nil;
    impl_->interfaces[interfaceNumber] = [[IOUSBHostInterface alloc] initWithIOService:service
        options:IOUSBHostObjectInitOptionsNone queue:nil error:&error interestHandler:nil];
    IOObjectRelease(service);
    return impl_->interfaces[interfaceNumber] != nil;
  }
}

void Session::close() {
  for (auto& interface : impl_->interfaces) {
    if (interface) { [interface destroy]; [interface release]; interface = nil; }
  }
  if (impl_->device) { [impl_->device destroy]; [impl_->device release]; impl_->device = nil; }
}

bool Session::resetDevice() {
  if (!impl_->device) return false;
  for (auto& interface : impl_->interfaces) {
    if (interface) { [interface destroy]; [interface release]; interface = nil; }
  }
  NSError* error = nil;
  const bool ok = [impl_->device resetWithError:&error];
  if (!ok) std::fprintf(stderr, "IOUSBHost device reset: %s\n",
                        error ? error.localizedDescription.UTF8String : "unknown");
  close();
  return ok;
}

bool Session::selectAlternate(std::uint8_t interfaceNumber, std::uint8_t alternate) {
  if (!impl_->interfaces[interfaceNumber]) return false;
  @autoreleasepool {
    NSError* error = nil;
    return [impl_->interfaces[interfaceNumber] selectAlternateSetting:alternate error:&error];
  }
}

bool Session::control(std::uint8_t requestType, std::uint8_t request, std::uint16_t value,
                      std::uint16_t index, std::vector<std::uint8_t>& data, int timeoutMs) {
  if (!impl_->device || data.size() > UINT16_MAX) return false;
  @autoreleasepool {
    IOUSBDeviceRequest packet{requestType, request, value, index,
                              static_cast<std::uint16_t>(data.size())};
    NSMutableData* buffer = [NSMutableData dataWithLength:data.size()];
    if ((requestType & 0x80) == 0 && !data.empty()) std::memcpy([buffer mutableBytes], data.data(), data.size());
    NSUInteger transferred = 0;
    NSError* error = nil;
    const BOOL ok = [impl_->device sendDeviceRequest:packet data:buffer bytesTransferred:&transferred
        completionTimeout:timeoutMs / 1000.0 error:&error];
    if (!ok) return false;
    if (requestType & 0x80) {
      data.resize(std::min<std::size_t>(transferred, data.size()));
      if (!data.empty()) std::memcpy(data.data(), [buffer bytes], data.size());
    }
    return true;
  }
}

bool Session::bulk(std::uint8_t endpoint, std::vector<std::uint8_t>& data,
                   std::size_t& transferred, int timeoutMs) {
  transferred = 0;
  @autoreleasepool {
    NSError* error = nil;
    IOUSBHostPipe* pipe = nil;
    for (auto interface : impl_->interfaces) {
      if (interface) { pipe = [interface copyPipeWithAddress:endpoint error:&error]; if (pipe) break; }
    }
    if (!pipe) return false;
    NSMutableData* buffer = [NSMutableData dataWithLength:data.size()];
    if ((endpoint & 0x80) == 0 && !data.empty()) std::memcpy([buffer mutableBytes], data.data(), data.size());
    NSUInteger count = 0;
    const BOOL ok = [pipe sendIORequestWithData:buffer bytesTransferred:&count
        completionTimeout:timeoutMs / 1000.0 error:&error];
    [pipe release];
    if (!ok) return false;
    transferred = count;
    if (endpoint & 0x80) {
      data.resize(std::min<std::size_t>(count, data.size()));
      if (!data.empty()) std::memcpy(data.data(), [buffer bytes], data.size());
    }
    return true;
  }
}

bool Session::iso(std::uint8_t endpoint, std::uint32_t packetBytes,
                  std::vector<std::uint8_t>& data, std::vector<IsoPacket>& packets) {
  if (!packetBytes || data.empty() || data.size() % packetBytes) return false;
  const std::size_t count = data.size() / packetBytes;
  if (count > 1024) return false;
  @autoreleasepool {
    NSError* error = nil;
    IOUSBHostPipe* pipe = nil;
    for (auto interface : impl_->interfaces) {
      if (interface) { pipe = [interface copyPipeWithAddress:endpoint error:&error]; if (pipe) break; }
    }
    if (!pipe) return false;
    NSMutableData* buffer = [NSMutableData dataWithLength:data.size()];
    if ((endpoint & 0x80) == 0) std::memcpy([buffer mutableBytes], data.data(), data.size());
    std::vector<IOUSBHostIsochronousTransaction> transactions(count);
    for (std::size_t i = 0; i < count; ++i) {
      transactions[i].requestCount = packetBytes;
      transactions[i].offset = static_cast<std::uint32_t>(i * packetBytes);
      transactions[i].options = IOUSBHostIsochronousTransactionOptionsNone;
    }
    const BOOL ok = [pipe sendIORequestWithData:buffer transactionList:transactions.data()
        transactionListCount:count firstFrameNumber:0 options:0 error:&error];
    [pipe release];
    if (!ok) { std::fprintf(stderr, "IOUSBHost iso request failed: %s\n", error ? error.localizedDescription.UTF8String : "unknown"); return false; }
    if (endpoint & 0x80) std::memcpy(data.data(), [buffer bytes], data.size());
    packets.clear(); packets.reserve(count);
    for (const auto& transaction : transactions)
      packets.push_back({transaction.offset, transaction.completeCount, transaction.status});
    return true;
  }
}

std::uint32_t Session::maxIsoPacketBytes(std::uint8_t endpoint) {
  @autoreleasepool {
    NSError* error = nil;
    for (auto interface : impl_->interfaces) {
      if (!interface) continue;
      IOUSBHostPipe* pipe = [interface copyPipeWithAddress:endpoint error:&error];
      if (!pipe) continue;
      const IOUSBHostIOSourceDescriptors* descriptors = pipe.descriptors;
      const std::uint32_t value = descriptors ? descriptors->ssCompanionDescriptor.wBytesPerInterval : 0;
      [pipe release];
      if (value) return value;
    }
    return 0;
  }
}

struct IsoStream::Impl {
  struct Batch { std::vector<std::uint8_t> data; std::vector<IsoPacket> packets; };
  struct Slot {
    NSMutableData* buffer = nil;
    std::vector<IOUSBHostIsochronousTransaction> transactions;
  };
  IOUSBHostPipe* pipe = nil;
  std::array<Slot, 4> slots;
  std::mutex mutex;
  std::condition_variable changed;
  std::map<std::uint64_t, Batch> ready;
  std::uint64_t submitted = 0, consumed = 0;
  bool running = false;
  bool failed = false;
  unsigned consecutiveErrors = 0;
  unsigned outstanding = 0;
  std::uint32_t packetBytes = 0;
  void submit(Slot* slot) {
    std::uint64_t order = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running) return;
      ++outstanding;
      order = submitted++;
    }
    for (auto& transaction : slot->transactions) {
      transaction.requestCount = packetBytes;
      transaction.completeCount = 0;
    }
    NSError* error = nil;
    const BOOL ok = [pipe enqueueIORequestWithData:slot->buffer
        transactionList:slot->transactions.data()
        transactionListCount:slot->transactions.size()
        firstFrameNumber:0 options:IOUSBHostIsochronousTransferOptionsNone
        error:&error completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction* completed) {
          Batch batch;
          if (status == kIOReturnSuccess) {
            const auto* source = static_cast<const std::uint8_t*>([slot->buffer bytes]);
            const std::size_t capacity = [slot->buffer length];
            for (std::size_t i = 0; i < slot->transactions.size(); ++i) {
              const auto& packet = completed[i];
              if (packet.status != kIOReturnSuccess || !packet.completeCount ||
                  static_cast<std::size_t>(packet.offset) + packet.completeCount > capacity) continue;
              const auto offset = static_cast<std::uint32_t>(batch.data.size());
              batch.data.insert(batch.data.end(), source + packet.offset,
                                source + packet.offset + packet.completeCount);
              batch.packets.push_back({offset, packet.completeCount, 0});
            }
          }
          bool again = false;
          {
            std::lock_guard<std::mutex> lock(mutex);
            --outstanding;
            if (status != kIOReturnSuccess && running) {
              if (++consecutiveErrors == 1) std::fprintf(stderr, "IOUSBHost queued iso completion: 0x%x\n", status);
              if (consecutiveErrors >= 64) failed = true;
            }
            if (status == kIOReturnSuccess) consecutiveErrors = 0;
            if (running && order >= consumed) {
              ready.emplace(order, std::move(batch));
              if (ready.size() > 512) {
                const auto oldest = ready.begin()->first;
                ready.erase(ready.begin());
                if (consumed <= oldest) consumed = oldest + 1;
              }
            }
            again = running && !failed;
            changed.notify_all();
          }
          if (again) submit(slot);
        }];
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!ok) { --outstanding; failed = true; ready.emplace(order, Batch{});
                 std::fprintf(stderr, "IOUSBHost queued iso submission: %s\n",
                                          error ? error.localizedDescription.UTF8String : "unknown"); }
      changed.notify_all();
    }
  }
};

IsoStream::IsoStream() : impl_(std::make_unique<Impl>()) {}
IsoStream::~IsoStream() { stop(); }

bool IsoStream::start(Session& session, std::uint8_t endpoint, std::uint32_t packetBytes) {
  stop();
  if (!packetBytes) return false;
  NSError* error = nil;
  for (auto interface : session.impl_->interfaces) {
    if (interface) { impl_->pipe = [interface copyPipeWithAddress:endpoint error:&error]; if (impl_->pipe) break; }
  }
  if (!impl_->pipe) return false;
  impl_->packetBytes = packetBytes;
  impl_->running = true;
  impl_->failed = false;
  impl_->submitted = impl_->consumed = 0;
  for (auto& slot : impl_->slots) {
    slot.buffer = [[NSMutableData alloc] initWithLength:static_cast<NSUInteger>(packetBytes) * 8];
    slot.transactions.resize(8);
    for (std::size_t i = 0; i < 8; ++i) {
      slot.transactions[i].offset = static_cast<std::uint32_t>(i * packetBytes);
      slot.transactions[i].options = IOUSBHostIsochronousTransactionOptionsNone;
    }
    impl_->submit(&slot);
    if (impl_->failed) { stop(); return false; }
  }
  return true;
}

bool IsoStream::next(std::vector<std::uint8_t>& data, std::vector<IsoPacket>& packets, int timeoutMs) {
  std::unique_lock<std::mutex> lock(impl_->mutex);
  impl_->changed.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&] {
    return impl_->ready.count(impl_->consumed) != 0 || impl_->failed || !impl_->running;
  });
  auto found = impl_->ready.find(impl_->consumed);
  if (found == impl_->ready.end()) return false;
  auto batch = std::move(found->second);
  impl_->ready.erase(found);
  ++impl_->consumed;
  data = std::move(batch.data);
  packets = std::move(batch.packets);
  return true;
}

void IsoStream::stop() {
  if (!impl_->pipe) return;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->running = false;
    impl_->changed.notify_all();
  }
  NSError* error = nil;
  [impl_->pipe abortWithError:&error];
  {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->changed.wait(lock, [&] { return impl_->outstanding == 0; });
    impl_->ready.clear();
  }
  for (auto& slot : impl_->slots) { [slot.buffer release]; slot.buffer = nil; }
  [impl_->pipe release]; impl_->pipe = nil;
}
}
