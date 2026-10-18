import os

Import("env")

env.BuildSources(
    os.path.join("$BUILD_DIR", "tap_fw"),
    os.path.join("$PROJECT_DIR", "src_fw", "src"),
)
env.BuildSources(
    os.path.join("$BUILD_DIR", "tap_control_core"),
    os.path.join("$PROJECT_DIR", "src", "tap_control", "core", "src"),
)
