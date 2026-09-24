cmake_minimum_required(VERSION 3.20)

add_library(tap_fw STATIC
  src/BusTransport.cpp
  src/BleCommandEndpoint.cpp
  src/DabbleGamepadMapper.cpp
  src/ControllerActuatorBridge.cpp
  src/FirmwareControlLoop.cpp)
target_include_directories(tap_fw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tap_fw PUBLIC cxx_std_17)
target_link_libraries(tap_fw PUBLIC tap_control_core tap_control_hal)
set_target_properties(tap_fw PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tap_fw PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(controller_actuator_bridge_tests
    tests/controller_actuator_bridge_tests.cpp)
  target_link_libraries(controller_actuator_bridge_tests PRIVATE tap_fw)
  if(MSVC)
    target_compile_options(controller_actuator_bridge_tests PRIVATE /UNDEBUG)
  else()
    target_compile_options(controller_actuator_bridge_tests PRIVATE -UNDEBUG)
  endif()
  add_test(NAME controller_actuator_bridge_tests
           COMMAND controller_actuator_bridge_tests)
  add_executable(ble_command_endpoint_tests
    tests/ble_command_endpoint_tests.cpp)
  target_link_libraries(ble_command_endpoint_tests PRIVATE tap_fw)
  if(MSVC)
    target_compile_options(ble_command_endpoint_tests PRIVATE /UNDEBUG)
  else()
    target_compile_options(ble_command_endpoint_tests PRIVATE -UNDEBUG)
  endif()
  add_test(NAME ble_command_endpoint_tests
           COMMAND ble_command_endpoint_tests)
endif()
