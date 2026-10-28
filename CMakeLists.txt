cmake_minimum_required(VERSION 3.20)
project(tap_controller LANGUAGES CXX)

# Controller host build, independent of desktop middleware.
# Enable TAP_BUILD_SIMULATOR explicitly to include the MuJoCo runners.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
include(CTest)
add_subdirectory(controller/hal)
add_subdirectory(controller/core)
add_subdirectory(controller/v3)
option(TAP_BUILD_SIMULATOR "Build MuJoCo simulator and its contracts" OFF)
if(TAP_BUILD_SIMULATOR)
  add_subdirectory(simulation)
endif()
