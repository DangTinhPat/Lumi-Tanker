cmake_minimum_required(VERSION 3.20)
project(tap_native LANGUAGES CXX)

# Canonical native build.  The robot model and simulator are native MuJoCo
# assets; the build has no middleware dependency.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
include(CTest)
add_subdirectory(src/tap_control/hal)
add_subdirectory(src/tap_control/core)
add_subdirectory(src_fw)
option(TAP_BUILD_SIMULATOR "Build MuJoCo simulator and its contracts" ON)
if(TAP_BUILD_SIMULATOR)
  add_subdirectory(src/tap_sim)
endif()
