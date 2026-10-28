cmake_minimum_required(VERSION 3.20)
project(tap_gui_native NONE)

# Optional standalone installation for the Python IPC client. The canonical
# build is the root native CMake project; this helper has no middleware
# dependency.
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/simulator_gui.py
        DESTINATION bin)
install(DIRECTORY tap_gui DESTINATION share/tap_gui_native)

install(DIRECTORY config DESTINATION share/tap_gui_native)
