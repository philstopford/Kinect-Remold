cmake_minimum_required(VERSION 3.20)
project(KinectRemoldNativeMac LANGUAGES OBJCXX)
if(NOT APPLE)
  message(FATAL_ERROR "macOS is required")
endif()
set(CMAKE_OBJCXX_STANDARD 17)
add_executable(kinect-remold-native-usb NativeUsb.mm NativeUsbHost.mm)
target_link_libraries(kinect-remold-native-usb PRIVATE
  "-framework Foundation" "-framework IOKit" "-framework IOUSBHost")
