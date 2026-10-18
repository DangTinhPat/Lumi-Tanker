cmake_minimum_required(VERSION 3.20)
project(tap_gui_native NONE)

# Optional standalone installation for the Python IPC client. The canonical
# build is the root native CMake project; this helper has no middleware
# dependency.
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/tap_native_gui.py
        DESTINATION bin)
install(DIRECTORY gui DESTINATION share/tap_gui_native)
