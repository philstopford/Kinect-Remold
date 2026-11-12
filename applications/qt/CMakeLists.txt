cmake_minimum_required(VERSION 3.20)
project(KinectRemoldQt LANGUAGES CXX)
if(APPLE)
  enable_language(OBJCXX)
endif()
find_package(onnxruntime CONFIG QUIET)
if(NOT onnxruntime_FOUND)
  message(FATAL_ERROR "ONNX Runtime is required for Qt body, hand and Motion Capture tracking on every platform; set onnxruntime_DIR to its CMake package directory")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt6 6.4 REQUIRED COMPONENTS Core Widgets Network WebSockets)
qt_add_executable(KinectRemoldQt MACOSX_BUNDLE main.cpp MotionDetector.cpp SpatialAudio.cpp MjpegAviWriter.cpp DepthCalibration.cpp TsdfVolume.cpp ScannerIcp.cpp ScannerMeshWriter.cpp ScannerSegmentation.cpp MocapNetwork.cpp MocapWebSocket.cpp MocapRig.cpp MocapBvh.cpp LanPoseLink.cpp)
target_link_libraries(KinectRemoldQt PRIVATE Qt6::Core Qt6::Widgets Qt6::Network Qt6::WebSockets)
if(onnxruntime_FOUND)
  target_sources(KinectRemoldQt PRIVATE macos/OnnxBodyTracker.cpp OnnxHandTracker.cpp OnnxFaceTracker.cpp SynSkeletonTracker.cpp RgbdRegistration.cpp ArticulatedPoseFit.cpp)
  target_link_libraries(KinectRemoldQt PRIVATE onnxruntime::onnxruntime)
  target_compile_definitions(KinectRemoldQt PRIVATE REMOLD_HAS_ONNX=1)
endif()
if(APPLE)
  target_sources(KinectRemoldQt PRIVATE macos/VisionTracker.mm macos/VideoRecorder.mm)
  target_link_libraries(KinectRemoldQt PRIVATE "-framework Vision" "-framework CoreGraphics" "-framework Foundation" "-framework AVFoundation" "-framework CoreVideo" "-framework CoreMedia")
else()
  target_sources(KinectRemoldQt PRIVATE VideoRecorder.cpp)
endif()
target_include_directories(KinectRemoldQt PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/modules/kinect-xbox-360-remold/linux/source/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/modules/kinect-one-remold/shared")
target_compile_definitions(KinectRemoldQt PRIVATE
  REMOLD_RUNTIME_DIR="/tmp" REMOLD_CONFIG_PATH="/tmp/remold.conf" REMOLD_FIRMWARE_PATH="/tmp/uac.bin")
set_target_properties(KinectRemoldQt PROPERTIES
  MACOSX_BUNDLE_GUI_IDENTIFIER org.kinectremold.qt
  MACOSX_BUNDLE_BUNDLE_NAME "Kinect Remold")
if(APPLE AND TARGET kinect-remold-native-usb)
  add_dependencies(KinectRemoldQt kinect-remold-native-usb)
  add_custom_command(TARGET KinectRemoldQt POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "$<TARGET_FILE:kinect-remold-native-usb>"
      "$<TARGET_FILE_DIR:KinectRemoldQt>/kinect-remold-native-usb")
endif()
if(REMOLD_MODEL_DIR AND EXISTS "${REMOLD_MODEL_DIR}/movenet.onnx")
  if(APPLE)
    set(_remold_staged_models "$<TARGET_FILE_DIR:KinectRemoldQt>/../Resources/models")
  else()
    set(_remold_staged_models "$<TARGET_FILE_DIR:KinectRemoldQt>/models")
  endif()
  add_custom_command(TARGET KinectRemoldQt POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${_remold_staged_models}")
  foreach(_remold_model movenet.onnx rtmpose-hand.onnx face-landmarks.onnx)
    if(EXISTS "${REMOLD_MODEL_DIR}/${_remold_model}")
      add_custom_command(TARGET KinectRemoldQt POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${REMOLD_MODEL_DIR}/${_remold_model}" "${_remold_staged_models}/${_remold_model}")
    endif()
  endforeach()
  if(EXISTS "${CMAKE_SOURCE_DIR}/third_party/models/face-blendshapes.onnx")
    add_custom_command(TARGET KinectRemoldQt POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_SOURCE_DIR}/third_party/models/face-blendshapes.onnx"
        "${_remold_staged_models}/face-blendshapes.onnx")
  endif()
endif()
if(UNIX AND NOT APPLE)
  set_target_properties(KinectRemoldQt PROPERTIES BUILD_RPATH "$ORIGIN")
endif()
