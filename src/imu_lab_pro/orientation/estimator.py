"""Orientation estimation and coordinate conventions for IMU LAB Pro.

The aircraft/model frame is right-handed: +X forward, +Y left, +Z up.
Displayed aerospace angles are roll positive right-wing-down, pitch positive
nose-up, and yaw positive nose-right. Internally, rotations always map vectors
from the aircraft frame into the fixed Z-up world frame.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


AXIS_VECTORS = {
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]),
    "-Y": np.array([0.0, -1.0, 0.0]),
    "+Z": np.array([0.0, 0.0, 1.0]),
    "-Z": np.array([0.0, 0.0, -1.0]),
}


def sensor_mount_rotation(forward_axis: str, up_axis: str) -> Rotation:
    """Return the sensor-to-aircraft rotation for two mounting directions.

    ``forward_axis`` and ``up_axis`` name the signed sensor axes that physically
    point toward the aircraft nose and aircraft top respectively.
    """
    try:
        forward_sensor = AXIS_VECTORS[forward_axis.upper()]
        up_sensor = AXIS_VECTORS[up_axis.upper()]
    except KeyError as exc:
        raise ValueError(f"unknown signed sensor axis: {exc.args[0]}") from exc
    if abs(float(np.dot(forward_sensor, up_sensor))) > 1e-9:
        raise ValueError("sensor forward and up axes must be perpendicular")
    left_sensor = np.cross(up_sensor, forward_sensor)
    aircraft_basis_in_sensor = np.column_stack(
        (forward_sensor, left_sensor, up_sensor))
    return Rotation.from_matrix(aircraft_basis_in_sensor.T)


def rotation_from_aerospace(roll: float, pitch: float, yaw: float) -> Rotation:
    """Create an aircraft-to-world rotation from angles in degrees."""
    return Rotation.from_euler("ZYX", [-yaw, -pitch, roll], degrees=True)


def aerospace_from_rotation(rotation: Rotation) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees using the documented convention."""
    z_angle, y_angle, x_angle = rotation.as_euler("ZYX", degrees=True)
    return float(x_angle), float(-y_angle), float(-z_angle)


def tilt_from_acceleration(acceleration_g: np.ndarray) -> Rotation:
    """Calculate a zero-heading aircraft attitude from a gravity observation."""
    acceleration = np.asarray(acceleration_g, dtype=float)
    norm = float(np.linalg.norm(acceleration))
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("a non-zero finite acceleration vector is required")
    acceleration /= norm
    roll = math.atan2(acceleration[1], acceleration[2])
    pitch = math.atan2(
        acceleration[0], math.hypot(acceleration[1], acceleration[2]))
    return Rotation.from_euler("ZYX", [0.0, -pitch, roll])


def propagate_orientation(
    orientation: Rotation, body_rate_rad_s: np.ndarray, dt_s: float
) -> Rotation:
    """Integrate a body-frame angular-rate vector with a quaternion update."""
    if not 0.0 < dt_s < 1.0:
        return orientation
    delta = np.asarray(body_rate_rad_s, dtype=float) * dt_s
    return orientation * Rotation.from_rotvec(delta)


@dataclass(frozen=True)
class AttitudeState:
    orientation: Rotation
    sensor_to_aircraft: Rotation
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    stationary: bool
    bias_ready: bool
    gyro_bias_dps: np.ndarray
    status: str


class AttitudeEstimator:
    """Quaternion complementary estimator with stationary gyro calibration."""

    def __init__(
        self,
        sensor_to_aircraft: Rotation | None = None,
        correction_gain: float = 2.0,
        bias_duration_s: float = 2.0,
    ) -> None:
        self.sensor_to_aircraft = (
            Rotation.identity() if sensor_to_aircraft is None
            else sensor_to_aircraft
        )
        self.correction_gain = float(correction_gain)
        self.bias_duration_s = float(bias_duration_s)
        self.reset()

    def reset(self) -> None:
        self.orientation = Rotation.identity()
        self.initialized = False
        self.last_t_us: int | None = None
        self.gyro_bias_dps = np.zeros(3)
        self.bias_ready = False
        self._bias_start_us: int | None = None
        self._bias_sum = np.zeros(3)
        self._bias_count = 0
        self._bias_accel_reference: np.ndarray | None = None

    def _reset_bias_window(self) -> None:
        self._bias_start_us = None
        self._bias_sum.fill(0.0)
        self._bias_count = 0
        self._bias_accel_reference = None

    @staticmethod
    def _elapsed_us(current: int, previous: int) -> int:
        return (int(current) - int(previous)) & 0xFFFFFFFF

    def _update_bias(self, t_us: int, acceleration: np.ndarray,
                     gyro_sensor_dps: np.ndarray) -> None:
        if self.bias_ready:
            return
        acceleration_norm = float(np.linalg.norm(acceleration))
        finite = (
            np.all(np.isfinite(acceleration))
            and np.all(np.isfinite(gyro_sensor_dps))
        )
        plausible = (
            finite
            and abs(acceleration_norm - 1.0) <= 0.08
            and float(np.linalg.norm(gyro_sensor_dps)) <= 30.0
        )
        mean_gyro = (
            self._bias_sum / self._bias_count
            if self._bias_count else gyro_sensor_dps
        )
        gyro_stable = (
            self._bias_count < 5
            or float(np.linalg.norm(gyro_sensor_dps - mean_gyro)) <= 2.5
        )
        acceleration_unit = (
            acceleration / acceleration_norm
            if acceleration_norm > 1e-9 else acceleration
        )
        gravity_stable = (
            self._bias_accel_reference is None
            or float(np.dot(acceleration_unit, self._bias_accel_reference))
            >= math.cos(math.radians(3.0))
        )
        candidate = plausible and gyro_stable and gravity_stable
        if not candidate:
            self._reset_bias_window()
            return
        if self._bias_start_us is None:
            self._bias_start_us = int(t_us)
            self._bias_accel_reference = acceleration_unit.copy()
        self._bias_sum += gyro_sensor_dps
        self._bias_count += 1
        elapsed = self._elapsed_us(t_us, self._bias_start_us) * 1e-6
        if elapsed >= self.bias_duration_s and self._bias_count >= 20:
            self.gyro_bias_dps = self._bias_sum / self._bias_count
            self.bias_ready = True

    def update(self, t_us: int, acceleration_g: np.ndarray,
               gyro_sensor_dps: np.ndarray) -> AttitudeState:
        acceleration_sensor = np.asarray(acceleration_g, dtype=float)
        gyro_sensor = np.asarray(gyro_sensor_dps, dtype=float)
        acceleration = self.sensor_to_aircraft.apply(acceleration_sensor)
        self._update_bias(t_us, acceleration, gyro_sensor)

        acceleration_norm = float(np.linalg.norm(acceleration))
        corrected_sensor_gyro = gyro_sensor - self.gyro_bias_dps
        gyro_aircraft_dps = self.sensor_to_aircraft.apply(corrected_sensor_gyro)
        stationary = self.bias_ready and (
            abs(acceleration_norm - 1.0) <= 0.08
            and float(np.linalg.norm(gyro_aircraft_dps)) <= 1.5
        )

        if not self.initialized and acceleration_norm > 1e-9:
            self.orientation = tilt_from_acceleration(acceleration)
            self.initialized = True

        if self.initialized and self.last_t_us is not None:
            dt_s = self._elapsed_us(t_us, self.last_t_us) * 1e-6
            if 0.0 < dt_s < 0.1:
                body_rate = np.deg2rad(gyro_aircraft_dps)
                self.orientation = propagate_orientation(
                    self.orientation, body_rate, dt_s)
                if 0.85 < acceleration_norm < 1.15:
                    # Correct only observable tilt. Rebuilding with the current
                    # heading prevents gravity correction from inventing yaw.
                    roll, pitch, yaw = aerospace_from_rotation(self.orientation)
                    gravity_tilt = tilt_from_acceleration(acceleration)
                    gravity_roll, gravity_pitch, _ = aerospace_from_rotation(
                        gravity_tilt)
                    fraction = 1.0 - math.exp(-self.correction_gain * dt_s)
                    roll_error = (gravity_roll - roll + 180.0) % 360.0 - 180.0
                    pitch_error = gravity_pitch - pitch
                    self.orientation = rotation_from_aerospace(
                        roll + fraction * roll_error,
                        pitch + fraction * pitch_error,
                        yaw,
                    )

        self.last_t_us = int(t_us)
        roll, pitch, yaw = aerospace_from_rotation(self.orientation)
        if self.bias_ready:
            status = "Gyro bias calibrated"
        elif self._bias_start_us is None:
            status = "Hold sensor still to calibrate gyro bias"
        else:
            elapsed = self._elapsed_us(t_us, self._bias_start_us) * 1e-6
            status = f"Calibrating gyro bias: {min(elapsed / self.bias_duration_s, 1.0):.0%}"
        return AttitudeState(
            orientation=self.orientation,
            sensor_to_aircraft=self.sensor_to_aircraft,
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            stationary=stationary,
            bias_ready=self.bias_ready,
            gyro_bias_dps=self.gyro_bias_dps.copy(),
            status=status,
        )
