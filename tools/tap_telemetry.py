"""Compatibility import for gui/tap_gui/telemetry.py."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gui"))
from tap_gui import telemetry
sys.modules[__name__] = telemetry
