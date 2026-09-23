cmake_minimum_required(VERSION 3.20)

add_library(tap_control_hal INTERFACE)
target_include_directories(tap_control_hal INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tap_control_hal INTERFACE cxx_std_17)

include(CTest)
if(BUILD_TESTING)
  add_executable(motor_frame_codec_tests tests/motor_frame_codec_tests.cpp)
  target_link_libraries(motor_frame_codec_tests PRIVATE tap_control_hal)
  if(MSVC)
    target_compile_options(motor_frame_codec_tests PRIVATE /UNDEBUG)
  else()
    target_compile_options(motor_frame_codec_tests PRIVATE -UNDEBUG)
  endif()
  add_test(NAME motor_frame_codec_tests COMMAND motor_frame_codec_tests)
endif()
