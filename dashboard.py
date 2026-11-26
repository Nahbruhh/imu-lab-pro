"""Compatibility launcher for IMU LAB Pro.

The application now lives in the ``imu_lab_pro`` package. Keeping this
launcher preserves the command documented by earlier releases.
"""

import runpy


if __name__ == "__main__":
    runpy.run_module("imu_lab_pro.app", run_name="__main__")
