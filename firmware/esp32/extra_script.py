import os

Import("env")

env.BuildSources(
    os.path.join("$BUILD_DIR", "tap_fw"),
    os.path.join("$PROJECT_DIR", "controller", "v3", "src"),
)
env.BuildSources(
    os.path.join("$BUILD_DIR", "tap_control_core"),
    os.path.join("$PROJECT_DIR", "controller", "core", "src"),
)
