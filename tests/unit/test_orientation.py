import numpy as np
from scipy.spatial.transform import Rotation

from imu_lab_pro.orientation import (
    AttitudeEstimator,
    aerospace_from_rotation,
    propagate_orientation,
    rotation_from_aerospace,
    sensor_mount_rotation,
    tilt_from_acceleration,
)


def assert_vector(actual, expected, tolerance=1e-9):
    np.testing.assert_allclose(actual, expected, atol=tolerance, rtol=0.0)


def test_angle_round_trip_uses_documented_aircraft_convention():
    expected = np.array([25.0, 15.0, -35.0])
    actual = aerospace_from_rotation(rotation_from_aerospace(*expected))
    assert_vector(actual, expected)


def test_level_acceleration_produces_level_aircraft():
    attitude = tilt_from_acceleration(np.array([0.0, 0.0, 1.0]))
    assert_vector(aerospace_from_rotation(attitude), [0.0, 0.0, 0.0])


def test_positive_x_up_displays_nose_up():
    attitude = tilt_from_acceleration(np.array([1.0, 0.0, 0.0]))
    # The model nose (+X) must point toward fixed-world up (+Z).
    assert_vector(attitude.apply([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0])


def test_positive_roll_moves_right_wing_down():
    attitude = rotation_from_aerospace(30.0, 0.0, 0.0)
    # In the aircraft frame +Y is left, so the right wing is -Y.
    assert attitude.apply([0.0, -1.0, 0.0])[2] < 0.0


def test_mount_rotation_maps_selected_sensor_axes_to_aircraft():
    mounting = sensor_mount_rotation('+Y', '+Z')
    assert_vector(mounting.apply([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0])
    assert_vector(mounting.apply([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0])


def test_quaternion_update_composes_body_axis_rotations():
    attitude = Rotation.identity()
    attitude = propagate_orientation(attitude, np.deg2rad([90.0, 0.0, 0.0]), 0.5)
    attitude = propagate_orientation(attitude, np.deg2rad([0.0, 90.0, 0.0]), 0.5)
    expected = (
        Rotation.from_rotvec(np.deg2rad([45.0, 0.0, 0.0]))
        * Rotation.from_rotvec(np.deg2rad([0.0, 45.0, 0.0]))
    )
    assert_vector(attitude.as_quat(), expected.as_quat())


def test_stationary_window_calibrates_gyro_bias():
    estimator = AttitudeEstimator(bias_duration_s=0.2)
    bias = np.array([0.35, -0.20, 0.12])
    state = None
    for index in range(30):
        state = estimator.update(
            index * 10_000,
            np.array([0.0, 0.0, 1.0]),
            bias,
        )
    assert state is not None
    assert state.bias_ready
    assert state.stationary
    assert_vector(state.gyro_bias_dps, bias)


def test_stationary_window_accepts_large_constant_sensor_bias():
    estimator = AttitudeEstimator(bias_duration_s=0.2)
    bias = np.array([7.0, -6.0, 5.0])
    state = None
    for index in range(30):
        state = estimator.update(
            index * 10_000,
            np.array([0.0, 0.0, 1.0]),
            bias,
        )
    assert state is not None
    assert state.bias_ready
    assert state.stationary
    assert_vector(state.gyro_bias_dps, bias)


def test_motion_restarts_startup_bias_calibration():
    estimator = AttitudeEstimator(bias_duration_s=0.2)
    for index in range(15):
        estimator.update(index * 10_000, [0.0, 0.0, 1.0], [0.2, 0.1, 0.0])
    estimator.update(150_000, [0.0, 0.0, 1.0], [20.0, 0.0, 0.0])
    state = estimator.update(160_000, [0.0, 0.0, 1.0], [0.2, 0.1, 0.0])
    assert not state.bias_ready


def test_gravity_correction_converges_tilt_without_changing_heading():
    estimator = AttitudeEstimator(correction_gain=2.0)
    estimator.orientation = rotation_from_aerospace(20.0, -10.0, 30.0)
    estimator.initialized = True
    estimator.bias_ready = True
    estimator.last_t_us = 0
    state = None
    for index in range(1, 301):
        state = estimator.update(
            index * 10_000,
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
        )
    assert state is not None
    assert abs(state.roll_deg) < 0.1
    assert abs(state.pitch_deg) < 0.1
    assert abs(state.yaw_deg - 30.0) < 0.5
