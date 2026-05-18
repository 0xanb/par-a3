"""Unit tests for the collision-impact detector predicate.

Jerk = first difference of accel_x divided by sample dt. Threshold
8 m/s³ in a 50 ms window at 100 Hz. Tunable for floors with higher
noise.
"""
from __future__ import annotations

from par_anomaly.detectors import is_collision_impact


def test_empty_buffer_not_collision -> None:
 """Cold start — no samples yet means no decision."""
 assert not is_collision_impact([])


def test_short_buffer_not_collision -> None:
 """Buffer below window_n is "not enough data"."""
 assert not is_collision_impact([0.0, 0.0, 0.0])


def test_quiet_chassis_no_collision -> None:
 """Five samples at rest → zero jerk → no collision."""
 assert not is_collision_impact([0.0] * 10)


def test_gentle_acceleration_no_collision -> None:
 """Smooth ramp-up over 50 ms ≈ 1 m/s² × 0.01 s = 0.01 m/s per step.
 Jerk = 1 m/s³ — well below the 8 m/s³ threshold."""
 accel = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05]
 assert not is_collision_impact(accel)


def test_single_large_spike_is_collision -> None:
 """One sample jumps from 0 to 1.0 m/s² over 0.01 s → jerk = 100 m/s³.
 Well over the 8 m/s³ threshold."""
 accel = [0.0, 0.0, 0.0, 0.0, 1.0]
 assert is_collision_impact(accel)


def test_negative_spike_also_collision -> None:
 """A deceleration spike (chassis hits obstacle, accel goes negative)
 trips just like a positive one — the predicate uses ``abs``."""
 accel = [0.0, 0.0, 0.0, 0.0, -1.0]
 assert is_collision_impact(accel)


def test_sustained_vibration_below_threshold -> None:
 """High-frequency low-amplitude vibration (rumble strip simulation):
 accel oscillates ±0.05 m/s². Jerk between samples is 0.10 / 0.01 = 10 m/s³.
 Threshold is 8 m/s³ so this WOULD trip — but in real use we condition
 on the stall predicate too. This test pins the predicate's literal
 behaviour."""
 accel = [0.0, 0.05, -0.05, 0.05, -0.05, 0.05]
 assert is_collision_impact(accel)


def test_custom_threshold_respected -> None:
 """A noisier chassis profile can raise the threshold."""
 accel = [0.0, 0.0, 0.0, 0.0, 1.0] # jerk 100 m/s³
 assert not is_collision_impact(accel, jerk_threshold_m_s3=200.0)


def test_custom_dt_respected -> None:
 """Lower sample rate → same delta over more time → lower jerk."""
 accel = [0.0, 0.0, 0.0, 0.0, 1.0]
 # dt=0.1 s → jerk = 1.0 / 0.1 = 10 m/s³
 assert is_collision_impact(accel, sample_dt_s=0.1)
 # dt=1.0 s → jerk = 1.0 / 1.0 = 1 m/s³ → below threshold
 assert not is_collision_impact(accel, sample_dt_s=1.0)


def test_only_recent_window_considered -> None:
 """Old spikes outside the window should not trip the predicate.

 Buffer has a large spike at the start (long-since-resolved) followed
 by quiet data. The window_n=5 view sees only the quiet tail.
 """
 # Spike at index 0, then 20 samples of quiet
 accel = [0.0, 5.0] + [0.0] * 20
 assert not is_collision_impact(accel)
