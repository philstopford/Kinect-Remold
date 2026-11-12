#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <QByteArray>
#include <QString>
#include <algorithm>
#include <memory>
#include <string>
#ifdef __APPLE__
#include <onnxruntime/coreml_provider_factory.h>
#endif

namespace remold {
inline std::unique_ptr<Ort::Session> createOnnxSession(Ort::Env& env,
    const QString& path,int intraOpThreads=0,const char* overrideKey=nullptr,
    QString* activeProvider=nullptr) {
  QByteArray requested=overrideKey?qgetenv(overrideKey).toLower():QByteArray{};
  if(requested.isEmpty())requested=qgetenv("REMOLD_ONNX_PROVIDER").toLower();
  if(!requested.isEmpty()&&requested!="cpu"&&requested!="coreml"&&requested!="cuda")
    qWarning("Unknown REMOLD_ONNX_PROVIDER=%s; using CPU",requested.constData());
  const std::string provider=requested=="coreml"?"CoreMLExecutionProvider":
      requested=="cuda"?"CUDAExecutionProvider":"";
  auto make=[&](bool accelerated) {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if(intraOpThreads>0)options.SetIntraOpNumThreads(intraOpThreads);
    if(accelerated) {
      if(provider=="CUDAExecutionProvider") {
        OrtCUDAProviderOptions cuda{};
        options.AppendExecutionProvider_CUDA(cuda);
      } else if(provider=="CoreMLExecutionProvider") {
#ifdef __APPLE__
        Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CoreML(options,COREML_FLAG_CREATE_MLPROGRAM));
#else
        throw std::runtime_error("Core ML requires macOS");
#endif
      }
    }
#ifdef _WIN32
    const std::wstring nativePath=path.toStdWString();
    return std::make_unique<Ort::Session>(env,nativePath.c_str(),options);
#else
    return std::make_unique<Ort::Session>(env,path.toUtf8().constData(),options);
#endif
  };
  if(provider.empty()) {auto session=make(false);if(activeProvider)*activeProvider="CPU";return session;}
  try {
    const auto available=Ort::GetAvailableProviders();
    if(std::find(available.begin(),available.end(),provider)==available.end())
      throw std::runtime_error("provider is not available in this ONNX Runtime build");
    auto session=make(true);
    if(activeProvider)*activeProvider=provider=="CoreMLExecutionProvider"?"Core ML":"CUDA";
    qInfo("ONNX provider %s: %s",provider.c_str(),qPrintable(path));
    return session;
  } catch(const std::exception& e) {
    qWarning("ONNX provider %s failed (%s); using CPU for %s",
        provider.c_str(),e.what(),qPrintable(path));
    auto session=make(false);if(activeProvider)*activeProvider="CPU (fallback)";return session;
  }
}
}
