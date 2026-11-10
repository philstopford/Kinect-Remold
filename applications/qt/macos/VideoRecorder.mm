#import "VideoRecorder.hpp"
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#include <QElapsedTimer>

#include <cstdint>
#include <cstring>

struct VideoRecorder::Impl {
  AVAssetWriter* writer = nil;
  AVAssetWriterInput* input = nil;
  AVAssetWriterInputPixelBufferAdaptor* adaptor = nil;
  QString error;
  int width = 0, height = 0;
  QElapsedTimer clock;
  std::int64_t lastTimeNs = -1;
};

VideoRecorder::VideoRecorder() : impl_(std::make_unique<Impl>()) {}
VideoRecorder::~VideoRecorder() { stop(); }
bool VideoRecorder::active() const { return impl_->writer != nil; }
QString VideoRecorder::error() const { return impl_->error; }

bool VideoRecorder::start(const QString& path, int width, int height) {
  stop();
  impl_->error.clear();
  if (width <= 0 || height <= 0) { impl_->error = "Invalid video dimensions"; return false; }
  @autoreleasepool {
    NSString* file = [NSString stringWithUTF8String:path.toUtf8().constData()];
    NSURL* url = [NSURL fileURLWithPath:file];
    [[NSFileManager defaultManager] removeItemAtURL:url error:nil];
    NSError* error = nil;
    impl_->writer = [[AVAssetWriter alloc] initWithURL:url fileType:AVFileTypeMPEG4 error:&error];
    if (!impl_->writer) { impl_->error = QString::fromUtf8(error.localizedDescription.UTF8String); return false; }
    NSDictionary* settings = @{
      AVVideoCodecKey: AVVideoCodecTypeH264,
      AVVideoWidthKey: @(width), AVVideoHeightKey: @(height),
      AVVideoCompressionPropertiesKey: @{AVVideoAverageBitRateKey: @(8'000'000)}
    };
    impl_->input = [[AVAssetWriterInput alloc] initWithMediaType:AVMediaTypeVideo outputSettings:settings];
    impl_->input.expectsMediaDataInRealTime = YES;
    NSDictionary* attributes = @{
      (id)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA),
      (id)kCVPixelBufferWidthKey: @(width), (id)kCVPixelBufferHeightKey: @(height),
      (id)kCVPixelBufferIOSurfacePropertiesKey: @{}
    };
    impl_->adaptor = [[AVAssetWriterInputPixelBufferAdaptor alloc]
        initWithAssetWriterInput:impl_->input sourcePixelBufferAttributes:attributes];
    if (![impl_->writer canAddInput:impl_->input]) {
      impl_->error = "AVFoundation rejected the video input"; stop(); return false;
    }
    [impl_->writer addInput:impl_->input];
    if (![impl_->writer startWriting]) {
      impl_->error = QString::fromUtf8(impl_->writer.error.localizedDescription.UTF8String);
      stop(); return false;
    }
    [impl_->writer startSessionAtSourceTime:kCMTimeZero];
    impl_->width = width; impl_->height = height; impl_->lastTimeNs = -1;
    impl_->clock.start();
    return true;
  }
}

bool VideoRecorder::append(const QImage& frame) {
  if (!active()) return false;
  if (!impl_->input.readyForMoreMediaData) return true;
  @autoreleasepool {
    const QImage bgra = frame.scaled(impl_->width, impl_->height, Qt::IgnoreAspectRatio,
                                    Qt::FastTransformation).convertToFormat(QImage::Format_ARGB32);
    CVPixelBufferRef buffer = nullptr;
    if (CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, impl_->adaptor.pixelBufferPool,
                                          &buffer) != kCVReturnSuccess || !buffer) {
      impl_->error = "Could not allocate video frame"; return false;
    }
    CVPixelBufferLockBaseAddress(buffer, 0);
    auto* destination = static_cast<std::uint8_t*>(CVPixelBufferGetBaseAddress(buffer));
    const std::size_t stride = CVPixelBufferGetBytesPerRow(buffer);
    for (int y = 0; y < impl_->height; ++y)
      std::memcpy(destination + y * stride, bgra.constScanLine(y), impl_->width * 4);
    CVPixelBufferUnlockBaseAddress(buffer, 0);
    std::int64_t timeNs = impl_->clock.nsecsElapsed();
    if (timeNs <= impl_->lastTimeNs) timeNs = impl_->lastTimeNs + 1;
    impl_->lastTimeNs = timeNs;
    const BOOL ok = [impl_->adaptor appendPixelBuffer:buffer
                                withPresentationTime:CMTimeMake(timeNs, 1'000'000'000)];
    CVPixelBufferRelease(buffer);
    if (!ok) impl_->error = QString::fromUtf8(impl_->writer.error.localizedDescription.UTF8String);
    return ok;
  }
}

bool VideoRecorder::stop() {
  if (!active()) return true;
  bool finished = true;
  if (impl_->writer.status == AVAssetWriterStatusWriting) {
    [impl_->input markAsFinished];
    dispatch_semaphore_t done = dispatch_semaphore_create(0);
    [impl_->writer finishWritingWithCompletionHandler:^{ dispatch_semaphore_signal(done); }];
    finished = dispatch_semaphore_wait(done,
        dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC)) == 0;
    if (!finished) impl_->error = "Timed out finalizing video";
    else if (impl_->writer.status != AVAssetWriterStatusCompleted)
      impl_->error = QString::fromUtf8(impl_->writer.error.localizedDescription.UTF8String);
  }
  [impl_->adaptor release]; impl_->adaptor = nil;
  [impl_->input release]; impl_->input = nil;
  [impl_->writer release]; impl_->writer = nil;
  return finished && impl_->error.isEmpty();
}
