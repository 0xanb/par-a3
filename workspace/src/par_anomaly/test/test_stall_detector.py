"""Unit tests for the wheel-stall detector predicate.

Pure predicate: takes already-armed buffers (the wrapper applies the
cold-start arm gate). Asserts that ``cmd_avg > cmd_threshold.odom_avg < odom_threshold`` over the trailing N samples.
"""
from __future__ import annotations

from par_anomaly.detectors import is_stalled


def test_empty_buffers_not_stalled -> None:
 assert not is_stalled(recent_cmd_v=[], recent_odom_v=[])


def test_short_buffer_not_stalled -> None:
 """Both rings under N samples → "not enough data"."""
 assert not is_stalled(
 recent_cmd_v=[0.10] * 5,
 recent_odom_v=[0.0] * 5,
 )


def test_uneven_buffer_lengths_not_stalled -> None:
 """If either buffer is short, no decision."""
 assert not is_stalled(
 recent_cmd_v=[0.10] * 20,
 recent_odom_v=[0.0] * 5,
 )


def test_canonical_stall -> None:
 """20 samples of cmd=0.10 (forward) + odom=0.00 (no motion) → stall."""
 assert is_stalled(
 recent_cmd_v=[0.10] * 20,
 recent_odom_v=[0.0] * 20,
 )


def test_robot_moving_not_stalled -> None:
 """Same cmd, but chassis actually delivers motion."""
 assert not is_stalled(
 recent_cmd_v=[0.10] * 20,
 recent_odom_v=[0.08] * 20,
 )


def test_cmd_zero_not_stalled -> None:
 """If planner asked for zero motion, "not moving" is not a stall."""
 assert not is_stalled(
 recent_cmd_v=[0.0] * 20,
 recent_odom_v=[0.0] * 20,
 )


def test_cmd_below_threshold_not_stalled -> None:
 """A cmd avg of 0.04 m/s is below the default cmd_threshold of 0.05 —
 treat as "planner not really asking for motion"."""
 assert not is_stalled(
 recent_cmd_v=[0.04] * 20,
 recent_odom_v=[0.0] * 20,
 )


def test_odom_at_threshold_not_stalled -> None:
 """Odom right at the threshold (0.02) — predicate uses strict ``<``."""
 assert not is_stalled(
 recent_cmd_v=[0.10] * 20,
 recent_odom_v=[0.02] * 20,
 )


def test_one_motion_sample_in_buffer_not_stalled -> None:
 """Even one valid motion sample pulls the average above the threshold."""
 cmd = [0.10] * 20
 odom = [0.0] * 19 + [0.5] # one sample of strong motion
 # odom_avg = 0.5/20 = 0.025 > 0.02 → not stalled
 assert not is_stalled(recent_cmd_v=cmd, recent_odom_v=odom)


def test_custom_n_window -> None:
 """Smaller window for noisy / fast-decision contexts."""
 assert is_stalled(
 recent_cmd_v=[0.10] * 5,
 recent_odom_v=[0.0] * 5,
 n=5,
 )


def test_negative_cmd_does_not_stall -> None:
 """Reverse intent (v=-0.10) with zero odom — the predicate only fires
 for FORWARD stalls because ``cmd_avg > threshold`` requires positive.
 The watchdog handles forward; rear-stall detection is its own concern."""
 assert not is_stalled(
 recent_cmd_v=[-0.10] * 20,
 recent_odom_v=[0.0] * 20,
 )


def test_custom_thresholds_respected -> None:
 """Operator can tighten thresholds (e.g. slower-tier trial)."""
 assert is_stalled(
 recent_cmd_v=[0.03] * 20,
 recent_odom_v=[0.01] * 20,
 cmd_threshold=0.02,
 odom_threshold=0.02,
 )


# ---- is_wheel_impact (fast 6-sample window, ~300 ms) ----------------------


def test_wheel_impact_imports:
 from par_anomaly.detectors import is_wheel_impact

 assert callable(is_wheel_impact)


def test_wheel_impact_canonical:
 """Transient impact: 6 samples cmd>0, odom=0 → wheel_impact True
 (300 ms at 20 Hz)."""
 from par_anomaly.detectors import is_wheel_impact

 assert is_wheel_impact(
 recent_cmd_v=[0.06] * 6,
 recent_odom_v=[0.0] * 6,
 )


def test_wheel_impact_short_buffer_not_impact:
 """3 samples is below the 6-sample minimum."""
 from par_anomaly.detectors import is_wheel_impact

 assert not is_wheel_impact(
 recent_cmd_v=[0.06] * 3,
 recent_odom_v=[0.0] * 3,
 )


def test_wheel_impact_moving_chassis_not_impact:
 from par_anomaly.detectors import is_wheel_impact

 assert not is_wheel_impact(
 recent_cmd_v=[0.06] * 6,
 recent_odom_v=[0.05] * 6,
 )


def test_wheel_impact_fires_earlier_than_is_stalled:
 """Both predicates take the same shape; wheel_impact fires after 6
 samples (~300 ms) while is_stalled needs 20 (~1 s). Useful for
 transient hits where the chassis decelerates briefly."""
 from par_anomaly.detectors import is_stalled, is_wheel_impact

 # Only 6 samples available
 cmd = [0.06] * 6
 odom = [0.0] * 6
 assert is_wheel_impact(recent_cmd_v=cmd, recent_odom_v=odom)
 assert not is_stalled(recent_cmd_v=cmd, recent_odom_v=odom)
