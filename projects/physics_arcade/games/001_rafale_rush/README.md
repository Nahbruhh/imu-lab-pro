# Rafale Rush

A simple side-scrolling game controlled by MPU6050 pitch and acceleration.

## Controls

1. Connect the ESP32-S3.
2. Wait for the first valid IMU sample.
3. Hold the sensor at a comfortable neutral angle and select **Set neutral**.
4. Tilt the sensor nose-up to climb and nose-down to descend.
5. Select **Start** and fly through the gates.
6. Make a short forward-thrust gesture to activate a temporary speed boost.

The game uses relative pitch as a vertical-speed command. It does not integrate
accelerometer data into position and is not a flight-dynamics simulation.

The **Set neutral** button becomes available after the first valid IMU sample.
Gyro-bias calibration continues in the background and no longer blocks the
game. **Start** becomes available after neutral is captured and current IMU
data is arriving. The status line shows which condition is still pending.

## Run

From the repository root:

```powershell
.venv\Scripts\python.exe projects\physics_arcade\games\001_rafale_rush\game.py --port COM4
```

Replace `COM4` with the ESP32-S3 serial port. The default sensor mounting is
sensor +X toward the aircraft nose and sensor +Z upward. Use `--help` for
alternative mounting options.

## Default control response

- Neutral dead zone: ±3°
- Full command: ±20° relative pitch
- Smoothed vertical-speed response
- Automatic pause after serial-data loss
- Gravity-compensated forward-thrust detection
- One-second boost with filtering, hysteresis, and cooldown
- Optional pitch and boost inversion for different mounting directions
