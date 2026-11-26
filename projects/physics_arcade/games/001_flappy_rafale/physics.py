"""Pure control and collision physics for Flappy Rafale."""

from dataclasses import dataclass
import math


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def angle_delta_deg(angle: float, reference: float) -> float:
    """Shortest signed difference between two angles in degrees."""
    return (angle - reference + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class PitchControlSettings:
    dead_zone_deg: float = 3.0
    full_command_deg: float = 20.0
    maximum_speed_px_s: float = 270.0
    response_time_s: float = 0.16


def pitch_command(current_pitch_deg: float, neutral_pitch_deg: float,
                  settings: PitchControlSettings) -> tuple[float, float]:
    """Return normalized climb command and relative pitch.

    Positive command means climb. The dead zone is removed before scaling so
    control remains continuous at its boundary.
    """
    relative = angle_delta_deg(current_pitch_deg, neutral_pitch_deg)
    magnitude = abs(relative)
    if magnitude <= settings.dead_zone_deg:
        return 0.0, relative
    usable_range = max(
        settings.full_command_deg - settings.dead_zone_deg, 1e-9)
    scaled = (magnitude - settings.dead_zone_deg) / usable_range
    return math.copysign(clamp(scaled, 0.0, 1.0), relative), relative


def update_vertical_speed(current_speed_px_s: float, command: float,
                          dt_s: float,
                          settings: PitchControlSettings) -> float:
    """Smooth vertical speed toward the pitch-command target.

    Screen Y increases downward, so a positive climb command has a negative
    target screen velocity.
    """
    if dt_s <= 0.0:
        return current_speed_px_s
    target = -clamp(command, -1.0, 1.0) * settings.maximum_speed_px_s
    alpha = 1.0 - math.exp(-dt_s / max(settings.response_time_s, 1e-6))
    return current_speed_px_s + alpha * (target - current_speed_px_s)


def circle_hits_gate(center_x: float, center_y: float, radius: float,
                     gate_x: float, gate_width: float,
                     gap_top: float, gap_bottom: float) -> bool:
    """Conservative collision test for a circular aircraft envelope."""
    horizontal_overlap = (
        center_x + radius >= gate_x
        and center_x - radius <= gate_x + gate_width
    )
    if not horizontal_overlap:
        return False
    return center_y - radius < gap_top or center_y + radius > gap_bottom
