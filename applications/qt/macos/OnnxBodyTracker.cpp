#include "OnnxBodyTracker.hpp"

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <QPainter>
#include <QPen>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

struct OnnxBodyTracker::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "KinectRemoldQt"};
  std::unique_ptr<Ort::Session> session;
  QString error;
  std::string inputName, outputName;
  int inputSize = 0;
  bool rgba8 = false;
  std::array<BodyKeypoint, 17> points{};
};

OnnxBodyTracker::OnnxBodyTracker(const QString& modelPath) : impl_(std::make_unique<Impl>()) {
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
    impl_->inputName = impl_->session->GetInputNameAllocated(0, allocator).get();
    impl_->outputName = impl_->session->GetOutputNameAllocated(0, allocator).get();
    auto inputType = impl_->session->GetInputTypeInfo(0);
    auto outputType = impl_->session->GetOutputTypeInfo(0);
    const auto input = inputType.GetTensorTypeAndShapeInfo();
    const auto output = outputType.GetTensorTypeAndShapeInfo();
    const auto inShape = input.GetShape(), outShape = output.GetShape();
    const bool rgba8 = input.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8 &&
        inShape.size() == 4 && (inShape[0] == 1 || inShape[0] == -1) && inShape[3] == 4;
    const bool rgbInt32 = input.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32 &&
        inShape.size() == 4 && (inShape[0] == 1 || inShape[0] == -1) && inShape[3] == 3;
    if ((!rgba8 && !rgbInt32) ||
        inShape[1] < 1 || inShape[1] > 1024 || inShape[1] != inShape[2] ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        outShape != std::vector<std::int64_t>({1, 1, 17, 3}))
      {
        std::string detail = "Unexpected MoveNet contract: input type=" +
            std::to_string(input.GetElementType()) + " shape=";
        for (auto dimension : inShape) detail += std::to_string(dimension) + ",";
        detail += " output type=" + std::to_string(output.GetElementType()) + " shape=";
        for (auto dimension : outShape) detail += std::to_string(dimension) + ",";
        throw std::runtime_error(detail);
      }
    impl_->inputSize = static_cast<int>(inShape[1]);
    impl_->rgba8 = rgba8;
  } catch (const std::exception& e) {
    impl_->session.reset();
    impl_->error = QString::fromUtf8(e.what());
  }
}

OnnxBodyTracker::~OnnxBodyTracker() = default;
bool OnnxBodyTracker::ready() const { return bool(impl_->session); }
QString OnnxBodyTracker::error() const { return impl_->error; }
std::array<BodyKeypoint, 17> OnnxBodyTracker::keypoints() const { return impl_->points; }

QImage OnnxBodyTracker::annotate(const QImage& image) {
  impl_->points = {};
  if (!ready() || image.isNull()) return image;
  try {
    const int size = impl_->inputSize;
    const QImage rgb = image.scaled(size, size, Qt::IgnoreAspectRatio, Qt::FastTransformation)
                           .convertToFormat(impl_->rgba8 ? QImage::Format_RGBA8888 : QImage::Format_RGB888);
    const std::array<std::int64_t, 4> shape{1, size, size, impl_->rgba8 ? 4 : 3};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value tensor{nullptr};
    std::vector<std::uint8_t> rgba;
    std::vector<std::int32_t> intRgb;
    if (impl_->rgba8) {
      rgba.resize(static_cast<std::size_t>(size) * size * 4);
      for (int y = 0; y < size; ++y)
        std::memcpy(rgba.data() + static_cast<std::size_t>(y) * size * 4, rgb.constScanLine(y), size * 4);
      tensor = Ort::Value::CreateTensor<std::uint8_t>(memory, rgba.data(), rgba.size(), shape.data(), shape.size());
    } else {
      intRgb.resize(static_cast<std::size_t>(size) * size * 3);
      for (int y = 0; y < size; ++y) {
        const auto* row = rgb.constScanLine(y);
        for (int x = 0; x < size * 3; ++x) intRgb[static_cast<std::size_t>(y) * size * 3 + x] = row[x];
      }
      tensor = Ort::Value::CreateTensor<std::int32_t>(memory, intRgb.data(), intRgb.size(), shape.data(), shape.size());
    }
    const char* inputName = impl_->inputName.c_str();
    const char* outputName = impl_->outputName.c_str();
    auto result = impl_->session->Run(Ort::RunOptions{nullptr}, &inputName, &tensor, 1, &outputName, 1);
    if (result.size() != 1 || !result[0].IsTensor() ||
        result[0].GetTensorTypeAndShapeInfo().GetElementCount() != 51) return image;
    const float* points = result[0].GetTensorData<float>();
    QImage output = image.copy();
    QPainter painter(&output);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(QColor(48, 239, 177), 3));
    painter.setBrush(QColor(48, 239, 177));
    for (int i = 0; i < 17; ++i) {
      const float y = points[i * 3], x = points[i * 3 + 1], confidence = points[i * 3 + 2];
      if (confidence >= 0.14f && std::isfinite(x) && std::isfinite(y)) {
        impl_->points[i] = {x, y, confidence};
        painter.drawEllipse(QPointF(x * output.width(), y * output.height()), 5, 5);
      }
    }
    return output;
  } catch (const std::exception& e) {
    impl_->error = QString::fromUtf8(e.what());
    return image;
  }
}
