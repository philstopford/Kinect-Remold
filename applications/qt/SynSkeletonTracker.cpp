#include "SynSkeletonTracker.hpp"

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int inputWidth = 320, inputHeight = 240;
quint16 mmAt(const QByteArray& data, int index) {
  const auto* p = reinterpret_cast<const quint8*>(data.constData() + index * 2);
  return quint16(p[0] | (quint16(p[1]) << 8));
}
}

struct SynSkeletonTracker::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "SynSkeleton"};
  std::unique_ptr<Ort::Session> session;
  QString error;
};

SynSkeletonTracker::SynSkeletonTracker(const QString& modelPath) : impl_(std::make_unique<Impl>()) {
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
    const auto inputName = impl_->session->GetInputNameAllocated(0, allocator);
    const auto inputType = impl_->session->GetInputTypeInfo(0);
    const auto input = inputType.GetTensorTypeAndShapeInfo();
    bool foundOutput = false;
    for (size_t i = 0; i < impl_->session->GetOutputCount(); ++i) {
      const auto name = impl_->session->GetOutputNameAllocated(i, allocator);
      if (QString::fromUtf8(name.get()) != "keypoints") continue;
      const auto outputType = impl_->session->GetOutputTypeInfo(i);
      const auto output = outputType.GetTensorTypeAndShapeInfo();
      const auto shape = output.GetShape();
      foundOutput = output.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
          shape.size() == 3 && (shape[0] == 1 || shape[0] == -1) &&
          shape[1] == jointCount && shape[2] == 4;
      break;
    }
    const auto inputShape = input.GetShape();
    if (QString::fromUtf8(inputName.get()) != "depth_metric" ||
        input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        inputShape.size() != 4 || (inputShape[0] != 1 && inputShape[0] != -1) ||
        inputShape[1] != 1 || inputShape[2] != inputHeight || inputShape[3] != inputWidth ||
        !foundOutput)
      throw std::runtime_error("Unexpected SynSkeleton ONNX input/output contract");
  } catch (const std::exception& e) {
    impl_->session.reset(); impl_->error = QString::fromUtf8(e.what());
  }
}

SynSkeletonTracker::~SynSkeletonTracker() = default;
bool SynSkeletonTracker::ready() const { return bool(impl_->session); }
QString SynSkeletonTracker::error() const { return impl_->error; }

bool SynSkeletonTracker::infer(const QByteArray& depth, const DepthIntrinsics& camera,
    const DepthCalibrationProfile* calibration,
    std::array<SynSkeletonJoint, jointCount>& joints) {
  joints = {};
  if (!ready() || !camera.valid() || depth.size() != qsizetype(camera.width) * camera.height * 2)
    return false;
  try {
    std::vector<float> input(inputWidth * inputHeight, -2.0f);
    for (int y = 0; y < inputHeight; ++y) for (int x = 0; x < inputWidth; ++x) {
      float samples[4]; int count = 0;
      const bool half = camera.width == inputWidth * 2 && camera.height == inputHeight * 2;
      const int sx = half ? x * 2 : std::min(camera.width - 1, int((x + 0.5) * camera.width / inputWidth));
      const int sy = half ? y * 2 : std::min(camera.height - 1, int((y + 0.5) * camera.height / inputHeight));
      for (int dy = 0; dy < (half ? 2 : 1); ++dy) for (int dx = 0; dx < (half ? 2 : 1); ++dx) {
        const int px = sx + dx, py = sy + dy;
        quint16 mm = mmAt(depth, py * camera.width + px);
        if (calibration) mm = calibration->correct(px, py, mm);
        if (mm > 350 && mm < 6000) samples[count++] = float(mm);
      }
      if (count) {
        std::sort(samples, samples + count);
        const float mm = count & 1 ? samples[count / 2] :
            (samples[count / 2 - 1] + samples[count / 2]) * 0.5f;
        input[y * inputWidth + x] = std::clamp((mm - 2200.0f) / 1800.0f, -1.0f, 1.8f);
      }
    }
    const std::array<std::int64_t, 4> shape{1, 1, inputHeight, inputWidth};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), shape.data(), shape.size());
    const char* inputName = "depth_metric"; const char* outputName = "keypoints";
    auto result = impl_->session->Run(Ort::RunOptions{nullptr}, &inputName, &tensor, 1, &outputName, 1);
    if (result.size() != 1 || !result[0].IsTensor() ||
        result[0].GetTensorTypeAndShapeInfo().GetElementCount() != jointCount * 4) return false;
    const float* points = result[0].GetTensorData<float>();
    for (int i = 0; i < jointCount; ++i) {
      const float u = points[i * 4], v = points[i * 4 + 1];
      const float mm = points[i * 4 + 2], confidence = points[i * 4 + 3];
      const float threshold = i >= 16 ? 0.11f : i == 0 ? 0.20f : 0.14f;
      if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(mm) ||
          !std::isfinite(confidence) || u < 0 || u > 1 || v < 0 || v > 1 ||
          mm < 350 || mm > 6000 || confidence < threshold) continue;
      const int x = std::clamp(int(std::lround(u * (camera.width - 1))), 0, camera.width - 1);
      const int y = std::clamp(int(std::lround(v * (camera.height - 1))), 0, camera.height - 1);
      double rx = 0, ry = 0;
      if (!camera.ray(x, y, rx, ry)) continue;
      const float z = mm * 0.001f;
      joints[i] = {float(rx * z), float(ry * z), z, std::clamp(confidence, 0.0f, 1.0f), true};
    }
    impl_->error.clear(); return true;
  } catch (const std::exception& e) {
    impl_->error = QString::fromUtf8(e.what()); return false;
  }
}

QImage SynSkeletonTracker::annotate(const QImage& preview,
    const std::array<SynSkeletonJoint, jointCount>& joints, const DepthIntrinsics& camera) {
  QImage output = preview.convertToFormat(QImage::Format_RGB32);
  if (output.isNull()) return output;
  QPainter painter(&output);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(QColor(48, 239, 177), 2));
  painter.setBrush(QColor(48, 239, 177));
  for (const auto& joint : joints) if (joint.tracked) {
    const int x = int(std::lround((joint.x / joint.z * camera.fx + camera.cx) *
                                  output.width() / camera.width));
    const int y = int(std::lround((joint.y / joint.z * camera.fy + camera.cy) *
                                  output.height() / camera.height));
    if (x >= 0 && x < output.width() && y >= 0 && y < output.height())
      painter.drawEllipse(QPointF(x, y), 3, 3);
  }
  return output;
}
