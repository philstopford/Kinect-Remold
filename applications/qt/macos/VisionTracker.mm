#import "VisionTracker.hpp"
#import <CoreGraphics/CoreGraphics.h>
#import <Vision/Vision.h>

#include <QPainter>
#include <QPen>

namespace {
QPointF imagePoint(CGPoint point, int width, int height) {
  return {point.x * width, (1.0 - point.y) * height};
}

void drawRecognizedPoints(QPainter& painter, NSDictionary* points, int width, int height) {
  for (id key in points) {
    VNRecognizedPoint* point = [points objectForKey:key];
    if (point.confidence < 0.3f) continue;
    painter.drawEllipse(imagePoint(point.location, width, height), 5, 5);
  }
}
}

QImage annotateWithVision(const QImage& frame, VisionOverlay overlay) {
  if (frame.isNull()) return frame;
  @autoreleasepool {
    QImage rgba = frame.scaled(960, 540, Qt::KeepAspectRatio, Qt::FastTransformation)
                      .convertToFormat(QImage::Format_RGBX8888);
    CGDataProviderRef provider = CGDataProviderCreateWithData(
        nullptr, rgba.constBits(), static_cast<std::size_t>(rgba.sizeInBytes()), nullptr);
    if (!provider) return frame;
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGImageRef image = CGImageCreate(rgba.width(), rgba.height(), 8, 32, rgba.bytesPerLine(),
        colorSpace, kCGBitmapByteOrderDefault | kCGImageAlphaNoneSkipLast,
        provider, nullptr, false, kCGRenderingIntentDefault);
    CGColorSpaceRelease(colorSpace);
    CGDataProviderRelease(provider);
    if (!image) return frame;

    VNImageRequestHandler* handler = [[VNImageRequestHandler alloc] initWithCGImage:image options:@{}];
    VNImageBasedRequest* request = nil;
    switch (overlay) {
      case VisionOverlay::Body: request = [[VNDetectHumanBodyPoseRequest alloc] init]; break;
      case VisionOverlay::Hands: request = [[VNDetectHumanHandPoseRequest alloc] init]; break;
      case VisionOverlay::Face: request = [[VNDetectFaceLandmarksRequest alloc] init]; break;
    }
    NSError* error = nil;
    const BOOL success = [handler performRequests:@[request] error:&error];
    QImage output = frame.copy();
    if (success) {
      QPainter painter(&output);
      painter.setRenderHint(QPainter::Antialiasing);
      painter.setPen(QPen(QColor(48, 239, 177), 3));
      painter.setBrush(QColor(48, 239, 177));
      if (overlay == VisionOverlay::Body) {
        for (VNHumanBodyPoseObservation* observation in request.results) {
          NSDictionary* points = [observation recognizedPointsForJointsGroupName:
              VNHumanBodyPoseObservationJointsGroupNameAll error:&error];
          if (points) drawRecognizedPoints(painter, points, output.width(), output.height());
        }
      } else if (overlay == VisionOverlay::Hands) {
        for (VNHumanHandPoseObservation* observation in request.results) {
          NSDictionary* points = [observation recognizedPointsForJointsGroupName:
              VNHumanHandPoseObservationJointsGroupNameAll error:&error];
          if (points) drawRecognizedPoints(painter, points, output.width(), output.height());
        }
      } else {
        for (VNFaceObservation* observation in request.results) {
          const CGRect box = observation.boundingBox;
          const VNFaceLandmarkRegion2D* region = observation.landmarks.allPoints;
          if (!region) continue;
          const CGPoint* points = region.normalizedPoints;
          for (NSUInteger i = 0; i < region.pointCount; ++i) {
            const CGPoint point = CGPointMake(box.origin.x + points[i].x * box.size.width,
                                              box.origin.y + points[i].y * box.size.height);
            painter.drawEllipse(imagePoint(point, output.width(), output.height()), 3, 3);
          }
        }
      }
    }
    [request release];
    [handler release];
    CGImageRelease(image);
    return output;
  }
}
