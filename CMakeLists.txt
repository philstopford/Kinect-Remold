cmake_minimum_required(VERSION 3.20)
project(KinectRemold LANGUAGES CXX)

# Expose the Qt application as a normal CLion CMake target when the repository
# root is opened as the project. Platform packaging remains in the scripts.
# The packaging scripts download and verify the inference models in a local
# build cache. Stage those same files in a direct CLion build when available.
set(REMOLD_MODEL_DIR "" CACHE PATH "Directory containing verified Kinect Remold ONNX models")
if(NOT REMOLD_MODEL_DIR)
  if(EXISTS "${CMAKE_SOURCE_DIR}/binaries/model-cache/movenet.onnx")
    set(_remold_default_models "${CMAKE_SOURCE_DIR}/binaries/model-cache")
  elseif(DEFINED ENV{REMOLD_WORK_ROOT})
    set(_remold_default_models "$ENV{REMOLD_WORK_ROOT}/models")
  elseif(DEFINED ENV{TMPDIR})
    set(_remold_default_models "$ENV{TMPDIR}/kinect-remold-build/models")
  else()
    set(_remold_default_models "/tmp/kinect-remold-build/models")
  endif()
  if(EXISTS "${_remold_default_models}/movenet.onnx")
    set(REMOLD_MODEL_DIR "${_remold_default_models}" CACHE PATH "Directory containing verified Kinect Remold ONNX models" FORCE)
  endif()
endif()
if(APPLE)
  add_subdirectory(drivers/macos)
endif()
add_subdirectory(applications/qt)
