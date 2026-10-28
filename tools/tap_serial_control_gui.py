#!/usr/bin/env python3
"""Compatibility entry point; the robot GUI lives in gui/robot_gui.py."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gui"))
import robot_gui

if __name__ == "__main__":
    robot_gui.main()
else:
    # Keep imports and socket-ownership patches bound to the actual module.
    sys.modules[__name__] = robot_gui
