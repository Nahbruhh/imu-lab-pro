# IMU LAB Pro v2

Engineering dashboard for an ESP32-S3 and MPU6050 connected over USB serial.

## Repository layout

```text
imu_lab_pro/                 Canonical Python application package
firmware/imu_stream_pro_v2/ Canonical Arduino sketch
assets/models/              Rafale GLB and OBJ assets
samples/                    Captured measurement examples
tools/                      Experimental and manual diagnostic utilities
tests/                      Automated test suite
archive/                    Historical source retained for reference
dashboard.py                Backward-compatible launcher
```

The supported application is `imu_lab_pro/app.py`. The PyVista Rafale
dashboard in `tools/` remains experimental until it is integrated into the
canonical application.

## Wiring

| MPU6050 | ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO5 |
| SCL | GPIO4 |

## Installation

1. Upload `firmware/imu_stream_pro_v2/imu_stream_pro_v2.ino` using Arduino IDE.
2. Close Arduino Serial Monitor and Serial Plotter.
3. Install the application dependencies:

   ```powershell
   py -m pip install -r requirements.txt
   ```

4. Run either supported launcher:

   ```powershell
   py -m imu_lab_pro --port COM4
   # or, for compatibility
   py dashboard.py --port COM4
   ```

The default serial rate is 460800 baud.

## Experimental Rafale dashboard

The former `dashboard_ap.py` is preserved as
`tools/rafale_dashboard_experimental.py`. It uses PyVista/VTK to render the
Rafale model and requires the optional 3D dependencies:

```powershell
py -m pip install -r requirements-3d.txt
py tools\rafale_dashboard_experimental.py --port COM4
```

Replace `COM4` with the ESP32-S3 serial port. The default baud rate is 460800;
it can be overridden with `--baud`.

The Rafale display uses a right-handed aircraft frame: +X forward, +Y left,
and +Z up. Keep the sensor still for at least two seconds after connecting so
the dashboard can estimate gyroscope bias. The **Level / zero** action is
rejected until that stationary calibration has completed.

If the MPU6050 axes are not mounted as +X toward the nose and +Z toward the
aircraft top, specify the signed sensor axes. For example, when sensor +Y
points forward and sensor +Z points upward:

```powershell
py tools\rafale_dashboard_experimental.py --port COM4 --sensor-forward +Y --sensor-up +Z
```

The forward and up selections must be perpendicular.

## Measurement limitations

- Available gyro ranges are ±250, ±500, ±1000, and ±2000 °/s.
- Available accelerometer ranges are ±2, ±4, ±8, and ±16 g.
- Acquisition currently uses scheduled I²C polling, not a verified MPU6050
  data-ready or FIFO acquisition path.
- Missing serial sequence IDs do not independently prove missing MPU6050
  internal samples.
- Measurements currently include sensor bias, scale error, gravity, and
  mounting alignment error. Calibration work is part of Phase 2.
- Effective bandwidth depends on the selected digital low-pass filter.
- Hardware validation results must be supported by captured test evidence.
