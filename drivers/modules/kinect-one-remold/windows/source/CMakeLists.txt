cmake_minimum_required(VERSION 3.20)
project(KinectOneRemold LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(KinectOneRemoldService KinectOneRemold.cpp)
if(MSVC)
  # The installed service must not depend on a developer's VC runtime install.
  set_property(TARGET KinectOneRemoldService PROPERTY
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
target_include_directories(KinectOneRemoldService PRIVATE ../../shared)
target_compile_definitions(KinectOneRemoldService PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(KinectOneRemoldService PRIVATE winusb setupapi cfgmgr32 advapi32)
