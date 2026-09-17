cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(tap_control_core_standalone LANGUAGES CXX)
endif()

set(TAP_MPC_MAX_HORIZON "80" CACHE STRING "Compiled MPC horizon capacity (4..80)")
if(NOT TAP_MPC_MAX_HORIZON MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "TAP_MPC_MAX_HORIZON must be an integer in [4,80]")
endif()
if(TAP_MPC_MAX_HORIZON LESS 4 OR TAP_MPC_MAX_HORIZON GREATER 80)
  message(FATAL_ERROR "TAP_MPC_MAX_HORIZON must be an integer in [4,80]")
endif()

add_library(tap_control_core STATIC
  src/EmbeddedController.cpp
  src/FlightObserver.cpp
  src/JointMotionPlanner.cpp
  src/JumpController.cpp
  src/LandingController.cpp
  src/LandingImpedance.cpp
  src/LandingReflex.cpp
  src/MpcBalancer.cpp
  src/StateMachine.cpp
  src/StationKeeper.cpp
  src/TrajectoryObserver.cpp
  src/WholeBodyModel.cpp)
target_include_directories(tap_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tap_control_core PUBLIC cxx_std_17)
target_compile_definitions(tap_control_core PUBLIC
  TAP_MPC_MAX_HORIZON=${TAP_MPC_MAX_HORIZON})
set_target_properties(tap_control_core PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tap_control_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# The ROS parent owns test registration when this directory is embedded.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  include(CTest)
  option(TAP_BUILD_BENCHMARK "Build the existing host control benchmark" OFF)
  if(BUILD_TESTING)
    foreach(test core_allocation_tests core_tests flight_observer_tests station_keeper_tests
                 deep_squat_tests jump_tests jump_quality_tests wheel_frame_tests
                 landing_reflex_tests landing_impedance_tests landing_controller_tests)
      add_executable(${test} ../tests/${test}.cpp)
      target_link_libraries(${test} PRIVATE tap_control_core)
      if(MSVC)
        target_compile_options(${test} PRIVATE /UNDEBUG)
      else()
        target_compile_options(${test} PRIVATE -UNDEBUG)
      endif()
      add_test(NAME ${test} COMMAND ${test})
    endforeach()
    add_executable(mpc_equivalence_tests ../tests/mpc_equivalence_tests.cpp
      ../tests/mpc_reference/MpcBalancer.cpp)
    target_link_libraries(mpc_equivalence_tests PRIVATE tap_control_core)
    add_test(NAME mpc_equivalence_tests COMMAND mpc_equivalence_tests)
  endif()
  if(TAP_BUILD_BENCHMARK)
    add_executable(control_benchmark ../apps/control_benchmark.cpp
      ../tests/mpc_reference/MpcBalancer.cpp)
    target_link_libraries(control_benchmark PRIVATE tap_control_core)
  endif()
endif()
