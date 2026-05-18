"""Unit tests for the tilt-detector predicates.

The trip threshold is 8° in either axis; the release (level) threshold is
5°. Hysteresis is owned by the ROS wrapper (it tracks ``_tilted_since``
+ the release timer). Predicates here are stateless.
"""
from __future__ import annotations

import math

from par_anomaly.detectors import is_level, is_tilted, quat_to_roll_pitch


# ---- quat_to_roll_pitch -----------------------------------------------------


def test_quat_identity_is_level -> None:
 """w=1, x=y=z=0 → roll=0, pitch=0 (chassis perfectly level)."""
 roll, pitch = quat_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
 assert abs(roll) < 1e-6
 assert abs(pitch) < 1e-6


def test_quat_pure_pitch_45deg -> None:
 """A 45° pitch around the chassis y-axis (nose up).

 Quaternion for rot_y(theta): (qx, qy, qz, qw) = (0, sin(t/2), 0, cos(t/2)).
 """
 t = math.radians(45.0)
 qx, qy, qz, qw = 0.0, math.sin(t / 2.0), 0.0, math.cos(t / 2.0)
 roll, pitch = quat_to_roll_pitch(qx, qy, qz, qw)
 assert abs(roll) < 1e-6
 assert abs(pitch - math.radians(45.0)) < 1e-6


def test_quat_pure_roll_30deg -> None:
 """A 30° roll around the chassis x-axis (leaning right)."""
 t = math.radians(30.0)
 qx, qy, qz, qw = math.sin(t / 2.0), 0.0, 0.0, math.cos(t / 2.0)
 roll, pitch = quat_to_roll_pitch(qx, qy, qz, qw)
 assert abs(roll - math.radians(30.0)) < 1e-6
 assert abs(pitch) < 1e-6


# ---- is_tilted --------------------------------------------------------------


def test_level_chassis_not_tilted -> None:
 assert not is_tilted(roll_rad=0.0, pitch_rad=0.0)


def test_small_drift_not_tilted -> None:
 """1° of drift each way is normal BNO055 noise on a level floor."""
 assert not is_tilted(
 roll_rad=math.radians(1.0),
 pitch_rad=math.radians(1.0),
 )


def test_pitch_above_threshold_is_tilted -> None:
 """10° forward pitch → tripped."""
 assert is_tilted(
 roll_rad=0.0,
 pitch_rad=math.radians(10.0),
 )


def test_roll_above_threshold_is_tilted -> None:
 """10° roll to the right → tripped."""
 assert is_tilted(
 roll_rad=math.radians(10.0),
 pitch_rad=0.0,
 )


def test_negative_pitch_above_threshold_is_tilted -> None:
 """Tipping backward (negative pitch) trips just like nose-up."""
 assert is_tilted(
 roll_rad=0.0,
 pitch_rad=math.radians(-12.0),
 )


def test_boundary_at_threshold_not_tilted -> None:
 """Exactly 8° (the default threshold) is NOT yet tilted.

 The predicate uses strict ``>`` so the threshold is exclusive.
 Matches ``is_wedged`` convention in nd_planner: boundary is "not yet".
 """
 assert not is_tilted(
 roll_rad=math.radians(8.0),
 pitch_rad=0.0,
 )


def test_just_over_threshold_is_tilted -> None:
 """Just past the threshold should trip."""
 assert is_tilted(
 roll_rad=math.radians(8.1),
 pitch_rad=0.0,
 )


def test_custom_threshold_respected -> None:
 """A more permissive floor profile can raise the threshold."""
 assert not is_tilted(
 roll_rad=math.radians(10.0),
 pitch_rad=0.0,
 max_roll_rad=math.radians(15.0),
 max_pitch_rad=math.radians(15.0),
 )


# ---- is_level (release-side hysteresis predicate) --------------------------


def test_level_within_clear_band -> None:
 """Below 5° in both axes → level (hysteresis release allowed)."""
 assert is_level(
 roll_rad=math.radians(2.0),
 pitch_rad=math.radians(3.0),
 )


def test_between_clear_and_trip_not_level -> None:
 """6° pitch is below trip (8°) but above clear (5°) — still 'recovering',
 not yet level. Prevents oscillation at the threshold."""
 assert not is_level(
 roll_rad=0.0,
 pitch_rad=math.radians(6.0),
 )


def test_above_trip_obviously_not_level -> None:
 assert not is_level(
 roll_rad=math.radians(15.0),
 pitch_rad=0.0,
 )
