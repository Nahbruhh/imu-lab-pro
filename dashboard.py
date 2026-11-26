"""Compatibility launcher for IMU LAB Pro.

The application now lives in the ``imu_lab_pro`` package. Keeping this
launcher preserves the command documented by earlier releases.
"""

import runpy
import sys
from pathlib import Path


if __name__ == "__main__":
    source_root = Path(__file__).resolve().parent / "src"
    sys.path.insert(0, str(source_root))
    runpy.run_module("imu_lab_pro.ui.application", run_name="__main__")
