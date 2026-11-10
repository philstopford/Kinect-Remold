#include "OnnxHandTracker.hpp"

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct OnnxHandTracker::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "KinectRemoldHands"};
  std::unique_ptr<Ort::Session> session;
  QString error;
  std::string inputName;
  std::array<std::string, 2> outputNames;
  std::array<int, 2> outputLengths{};
  std::array<std::array<BodyKeypoint,21>,2> points{};
};

OnnxHandTracker::OnnxHandTracker(const QString& modelPath) : impl_(std::make_unique<Impl>()) {
  try {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef _WIN32
    const std::wstring nativePath = modelPath.toStdWString();
    impl_->session = std::make_unique<Ort::Session>(impl_->env, nativePath.c_str(), options);
#else
    impl_->session = std::make_unique<Ort::Session>(impl_->env, modelPath.toUtf8().constData(), options);
#endif
    Ort::AllocatorWithDefaultOptions allocator;
    if (impl_->session->GetInputCount() != 1 || impl_->session->GetOutputCount() < 2)
      throw std::runtime_error("Unexpected RTMPose Hand input/output count");
    impl_->inputName = impl_->session->GetInputNameAllocated(0, allocator).get();
    const auto inputType = impl_->session->GetInputTypeInfo(0);
    const auto input = inputType.GetTensorTypeAndShapeInfo();
    const auto shape = input.GetShape();
    if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        shape.size() != 4 || (shape[0] != 1 && shape[0] != -1) ||
        shape[1] != 3 || shape[2] != 256 || shape[3] != 256)
      {
        std::string detail = "Expected RTMPose Hand FLOAT [1,3,256,256]; got type " +
            std::to_string(input.GetElementType()) + " shape";
        for (auto dimension : shape) detail += " " + std::to_string(dimension);
        throw std::runtime_error(detail);
      }
    for (int i = 0; i < 2; ++i) {
      impl_->outputNames[i] = impl_->session->GetOutputNameAllocated(i, allocator).get();
      const auto outputType = impl_->session->GetOutputTypeInfo(i);
      const auto output = outputType.GetTensorTypeAndShapeInfo();
      const auto dims = output.GetShape();
      if (output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
          dims.size() != 3 || (dims[0] != 1 && dims[0] != -1) ||
          (dims[1] != 21 && dims[1] != -1) || dims[2] < 128 || dims[2] > 1024)
        {
          std::string detail = "Unexpected RTMPose Hand SimCC output: type " +
              std::to_string(output.GetElementType()) + " shape";
          for (auto dimension : dims) detail += " " + std::to_string(dimension);
          throw std::runtime_error(detail);
        }
      impl_->outputLengths[i] = int(dims[2]);
    }
  } catch (const std::exception& e) {
    impl_->session.reset(); impl_->error = QString::fromUtf8(e.what());
  }
}

OnnxHandTracker::~OnnxHandTracker() = default;
bool OnnxHandTracker::ready() const { return bool(impl_->session); }
QString OnnxHandTracker::error() const { return impl_->error; }
std::array<std::array<BodyKeypoint,21>,2> OnnxHandTracker::keypoints() const {
  return impl_->points;
}

QImage OnnxHandTracker::annotate(const QImage& image, const std::array<BodyKeypoint, 17>& body) {
  impl_->points = {};
  if (!ready() || image.isNull()) return image;
  QImage output = image.convertToFormat(QImage::Format_RGB32);
  QPainter painter(&output);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(QColor(255, 183, 63), 2));
  painter.setBrush(QColor(255, 183, 63));
  try {
    int handIndex=0;
    for (const auto [wristIndex, elbowIndex] : {std::pair{9, 7}, std::pair{10, 8}}) {
      const int currentHand=handIndex++;
      const auto wrist = body[wristIndex], elbow = body[elbowIndex];
      if (wrist.confidence < 0.3f || elbow.confidence < 0.3f) continue;
      const float wx = wrist.x * image.width(), wy = wrist.y * image.height();
      const float ex = elbow.x * image.width(), ey = elbow.y * image.height();
      const float forearm = std::hypot(wx - ex, wy - ey);
      if (!std::isfinite(forearm) || forearm < 12) continue;
      const float side = std::clamp(forearm * 1.55f, 72.0f, 190.0f);
      const float x0 = wx + (wx - ex) * 0.2f - side * 0.5f;
      const float y0 = wy + (wy - ey) * 0.2f - side * 0.5f;
      const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
      std::vector<float> pixels(3 * 256 * 256);
      constexpr float mean[]{123.675f, 116.28f, 103.53f};
      constexpr float stddev[]{58.395f, 57.12f, 57.375f};
      for (int c = 0; c < 3; ++c) for (int y = 0; y < 256; ++y) for (int x = 0; x < 256; ++x) {
        const int sx = std::clamp(int(std::lround(x0 + (x + .5f) * side / 256)), 0, rgb.width() - 1);
        const int sy = std::clamp(int(std::lround(y0 + (y + .5f) * side / 256)), 0, rgb.height() - 1);
        pixels[(c * 256 + y) * 256 + x] = (rgb.constScanLine(sy)[sx * 3 + c] - mean[c]) / stddev[c];
      }
      const std::array<std::int64_t, 4> shape{1, 3, 256, 256};
      auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      auto tensor = Ort::Value::CreateTensor<float>(memory, pixels.data(), pixels.size(), shape.data(), shape.size());
      const char* inputName = impl_->inputName.c_str();
      const char* outputNames[]{impl_->outputNames[0].c_str(), impl_->outputNames[1].c_str()};
      auto result = impl_->session->Run(Ort::RunOptions{nullptr}, &inputName, &tensor, 1, outputNames, 2);
      if (result.size() != 2 || !result[0].IsTensor() || !result[1].IsTensor()) continue;
      const auto xShape = result[0].GetTensorTypeAndShapeInfo().GetShape();
      const auto yShape = result[1].GetTensorTypeAndShapeInfo().GetShape();
      if (xShape.size() != 3 || yShape.size() != 3 || xShape[1] < 21 || yShape[1] < 21) {
        impl_->error = QString("RTMPose output has %1/%2 landmarks; 21 required")
            .arg(xShape.size() == 3 ? xShape[1] : -1).arg(yShape.size() == 3 ? yShape[1] : -1);
        return output;
      }
      const float* xx = result[0].GetTensorData<float>();
      const float* yy = result[1].GetTensorData<float>();
      for (int k = 0; k < 21; ++k) {
        int ix = 0, iy = 0; float maxX = -INFINITY, maxY = -INFINITY;
        for (int i = 0; i < impl_->outputLengths[0]; ++i)
          if (xx[k * impl_->outputLengths[0] + i] > maxX) { maxX = xx[k * impl_->outputLengths[0] + i]; ix = i; }
        for (int i = 0; i < impl_->outputLengths[1]; ++i)
          if (yy[k * impl_->outputLengths[1] + i] > maxY) { maxY = yy[k * impl_->outputLengths[1] + i]; iy = i; }
        const float confidence = 1.0f / (1.0f + std::exp(-std::min(maxX, maxY)));
        if (confidence < 0.45f) continue;
        const float px = x0 + ix * side / impl_->outputLengths[0];
        const float py = y0 + iy * side / impl_->outputLengths[1];
        if (px >= 0 && py >= 0 && px < image.width() && py < image.height()) {
          impl_->points[currentHand][k]={px/image.width(),py/image.height(),confidence};
          painter.drawEllipse(QPointF(px, py), 3, 3);
        }
      }
    }
    impl_->error.clear();
  } catch (const std::exception& e) { impl_->error = QString::fromUtf8(e.what()); }
  return output;
}
