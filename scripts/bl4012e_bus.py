"""Compile the BL4012e 3in1 bus component into the hybrid firmware."""
import os
Import("env")
base = os.path.join(env.subst("$PROJECT_DIR"), "drivers", "bl4012e")
env.Append(CPPPATH=[os.path.join(base, "include")])
env.BuildSources(os.path.join("$BUILD_DIR", "bl4012e_bus"), os.path.join(base, "src"))
