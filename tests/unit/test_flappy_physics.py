import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "projects"
    / "physics_arcade"
    / "games"
    / "001_flappy_rafale"
    / "physics.py"
)
SPEC = importlib.util.spec_from_file_location("flappy_rafale_physics", MODULE_PATH)
physics = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(physics)


def test_angle_delta_wraps_across_180_degrees():
    assert physics.angle_delta_deg(-179.0, 179.0) == pytest.approx(2.0)


def test_pitch_inside_dead_zone_is_neutral():
    settings = physics.PitchControlSettings()
    command, relative = physics.pitch_command(11.0, 10.0, settings)
    assert command == 0.0
    assert relative == pytest.approx(1.0)


def test_nose_up_and_down_commands_have_correct_sign():
    settings = physics.PitchControlSettings()
    climb, _ = physics.pitch_command(30.0, 10.0, settings)
    descend, _ = physics.pitch_command(-10.0, 10.0, settings)
    assert climb == pytest.approx(1.0)
    assert descend == pytest.approx(-1.0)


def test_climb_command_produces_upward_screen_velocity():
    settings = physics.PitchControlSettings()
    speed = physics.update_vertical_speed(0.0, 1.0, 0.1, settings)
    assert -settings.maximum_speed_px_s < speed < 0.0


def test_gate_collision_respects_open_gap():
    assert not physics.circle_hits_gate(
        center_x=100.0,
        center_y=200.0,
        radius=20.0,
        gate_x=90.0,
        gate_width=80.0,
        gap_top=150.0,
        gap_bottom=250.0,
    )
    assert physics.circle_hits_gate(
        center_x=100.0,
        center_y=140.0,
        radius=20.0,
        gate_x=90.0,
        gate_width=80.0,
        gap_top=150.0,
        gap_bottom=250.0,
    )
