cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(tap_control_core_standalone LANGUAGES CXX)
endif()

add_library(tap_control_core STATIC
  src/v3/LegHeight.cpp
  src/v3/RockerLower.cpp
  src/v3/V3Controller.cpp
  src/EmbeddedController.cpp
  src/ImuStateEstimator.cpp
  src/FlightObserver.cpp
  src/JointMotionPlanner.cpp
  src/JumpController.cpp
  src/LandingController.cpp
  src/LandingImpedance.cpp
  src/LandingReflex.cpp
  src/LqrBalancer.cpp
  src/StateMachine.cpp
  src/StationKeeper.cpp
  src/TorqueResidualEstimator.cpp
  src/TrajectoryObserver.cpp
  src/WholeBodyModel.cpp
  src/WholeBodyStanceController.cpp)
target_include_directories(tap_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tap_control_core PUBLIC cxx_std_17)
set_target_properties(tap_control_core PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tap_control_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Register the same deterministic core tests when embedded by the canonical
# native superbuild.
option(TAP_CONTROL_CORE_REGISTER_TESTS "Register portable core tests" ON)
include(CTest)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR OR
   TAP_CONTROL_CORE_REGISTER_TESTS)
  option(TAP_BUILD_BENCHMARK "Build the existing host control benchmark" OFF)
  if(BUILD_TESTING)
    foreach(test core_allocation_tests core_tests height_pose_tests imu_state_estimator_tests flight_observer_tests
                 torque_residual_estimator_tests station_keeper_tests
                 deep_squat_tests jump_tests jump_quality_tests wheel_frame_tests
                 landing_reflex_tests landing_impedance_tests landing_controller_tests
                 whole_body_stance_tests)
      add_executable(${test} ../tests/${test}.cpp)
      target_link_libraries(${test} PRIVATE tap_control_core)
      if(MSVC)
        target_compile_options(${test} PRIVATE /UNDEBUG)
      else()
        target_compile_options(${test} PRIVATE -UNDEBUG)
      endif()
      add_test(NAME ${test} COMMAND ${test})
    endforeach()
    add_executable(lqr_tests ../tests/lqr_tests.cpp)
    target_link_libraries(lqr_tests PRIVATE tap_control_core)
    add_test(NAME lqr_tests COMMAND lqr_tests)
    add_executable(v3_controller_tests ../tests/v3_controller_tests.cpp)
    target_link_libraries(v3_controller_tests PRIVATE tap_control_core)
    target_compile_options(v3_controller_tests PRIVATE -UNDEBUG)
    add_test(NAME v3_controller_tests COMMAND v3_controller_tests)
  endif()
endif()
