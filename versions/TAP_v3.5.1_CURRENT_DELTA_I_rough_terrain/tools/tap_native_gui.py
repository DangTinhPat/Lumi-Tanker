#!/usr/bin/env python3
"""Start the TAP native MuJoCo dashboard from a source checkout."""

from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "src/gui"))

from gui.native_dashboard import main  # noqa: E402


if __name__ == "__main__":
    main()
