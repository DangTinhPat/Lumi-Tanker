cmake_minimum_required(VERSION 3.20)
project(tap_sim_native LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# MuJoCo is a native dependency.  Prefer an explicit MUJOCO_ROOT, then find
# the wheel installed for the current Python.  No middleware is needed to
# build or run this target.
if(NOT MUJOCO_ROOT)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c
    "import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)"
    OUTPUT_VARIABLE MUJOCO_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE MUJOCO_PY_RESULT)
endif()
if(NOT MUJOCO_ROOT AND EXISTS "/home/dvt/.local/lib/python3.12/site-packages/mujoco/include/mujoco/mujoco.h")
  set(MUJOCO_ROOT "/home/dvt/.local/lib/python3.12/site-packages/mujoco")
endif()
if(NOT MUJOCO_ROOT OR NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo not found. Set MUJOCO_ROOT to the native MuJoCo installation.")
endif()
if(EXISTS "${MUJOCO_ROOT}/libmujoco.so.3.13.0")
  set(MUJOCO_LIBRARY "${MUJOCO_ROOT}/libmujoco.so.3.13.0")
else()
  find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco
    PATHS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH)
endif()
if(NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo shared library not found below ${MUJOCO_ROOT}")
endif()

if(NOT TARGET tap_control_core)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../controller/core tap_control_core_build)
endif()
if(NOT TARGET tap_control_hal)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../controller/hal tap_control_hal_build)
endif()

if(NOT TARGET tap_fw)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../controller/v3 tap_fw_build)
endif()

add_executable(tap_mujoco_native native/native_runner.cpp)
target_compile_features(tap_mujoco_native PRIVATE cxx_std_17)
target_compile_options(tap_mujoco_native PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tap_mujoco_native PRIVATE
  "${MUJOCO_ROOT}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/drivers")
target_link_libraries(tap_mujoco_native PRIVATE
  tap_control_core tap_control_hal "${MUJOCO_LIBRARY}" Threads::Threads dl)
set_target_properties(tap_mujoco_native PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")

# The firmware V3Controller in MuJoCo on the GIM6010-8 model (gim_dynamic.xml).
add_executable(tap_mujoco_gim_v3 native/gim_v3_runner.cpp)
target_compile_features(tap_mujoco_gim_v3 PRIVATE cxx_std_17)
target_compile_options(tap_mujoco_gim_v3 PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tap_mujoco_gim_v3 PRIVATE
  "${MUJOCO_ROOT}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/drivers")
target_link_libraries(tap_mujoco_gim_v3 PRIVATE
  tap_fw tap_control_core tap_control_hal "${MUJOCO_LIBRARY}" Threads::Threads dl)
set_target_properties(tap_mujoco_gim_v3 PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")

include(CTest)

install(TARGETS tap_mujoco_native tap_mujoco_gim_v3 RUNTIME DESTINATION bin)
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/validate_model.py
  scripts/mujoco_sensor_actuator_backend.py
  tests/native_runner_contract_test.py
  tests/native_runner_lifecycle_test.py
  tests/native_drive_stop_guard_test.py
  tests/native_stop_balance_test.py
  tests/native_leg_speed_loop_test.py
  tests/native_neutral_balance_test.py
  tests/native_assist_mechanism_test.py
  tests/native_yaw_station_test.py
  tests/native_posture_transition_test.py
  tests/native_dance_skill_test.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(tap_mujoco_sim_actuator_bus_tests tests/sim_actuator_bus_tests.cpp)
  target_compile_features(tap_mujoco_sim_actuator_bus_tests PRIVATE cxx_std_17)
  target_compile_options(tap_mujoco_sim_actuator_bus_tests PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(tap_mujoco_sim_actuator_bus_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers")
  target_link_libraries(tap_mujoco_sim_actuator_bus_tests PRIVATE
    tap_control_core tap_control_hal)
  add_test(NAME tap_mujoco_sim_actuator_bus
           COMMAND tap_mujoco_sim_actuator_bus_tests)
  add_test(
    NAME tap_mujoco_model_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_model.py
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  set_tests_properties(tap_mujoco_model_contract PROPERTIES
    ENVIRONMENT "PYTHONPATH=$ENV{PYTHONPATH}")
  add_test(
    NAME tap_mujoco_backend_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/backend_contract_test.py
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  set_tests_properties(tap_mujoco_backend_contract PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}")
  add_test(
    NAME tap_mujoco_native_runner_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_runner_contract_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_runner_lifecycle
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_runner_lifecycle_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_drive_stop_guard
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_drive_stop_guard_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_neutral_balance
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_neutral_balance_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_stop_balance
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_stop_balance_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_leg_speed_loop
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_leg_speed_loop_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_assist_mechanism
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_assist_mechanism_test.py
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_yaw_station
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_yaw_station_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_posture_transition
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_posture_transition_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml)
  add_test(
    NAME tap_mujoco_native_dance_skill
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/native_dance_skill_test.py
            --runner $<TARGET_FILE:tap_mujoco_native>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/tap_robot.xml
            --duration-s 120)
  # Small vertical hop of the firmware controller in MuJoCo (docs/V3.7_HOP_20261021.md, RULES.md Rule 6). Sim evidence only.
  add_test(
    NAME tap_mujoco_gim_hop
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/gim_hop_test.py
            --runner $<TARGET_FILE:tap_mujoco_gim_v3>
            --model ${CMAKE_CURRENT_SOURCE_DIR}/model/gim_dynamic.xml
            --check --min-pass 11)
  # These tests own a native runner and sensor clock. Prevent a parallel CTest
  # job from turning host contention into a false lifecycle/control failure.
  set_tests_properties(
    tap_mujoco_native_runner_contract
    tap_mujoco_native_runner_lifecycle
    tap_mujoco_native_drive_stop_guard
    tap_mujoco_native_neutral_balance
    tap_mujoco_native_stop_balance
    tap_mujoco_native_leg_speed_loop
    tap_mujoco_native_yaw_station
    tap_mujoco_native_posture_transition
    tap_mujoco_native_dance_skill
    tap_mujoco_gim_hop
    PROPERTIES RUN_SERIAL TRUE)
endif()
