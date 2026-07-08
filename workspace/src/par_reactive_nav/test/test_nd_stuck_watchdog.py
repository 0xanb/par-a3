"""Regression tests for nd_planner stuck-watchdog (post , /cmd_vel-feedback design).

The watchdog detects the wedge by comparing PLANNER INTENT to ACTUAL /cmd_vel
post-safety-clamp. ND's classifier emits LS1 with v=0.07 indefinitely when one
side is close, but H1 ToF halo zeroes /cmd_vel to 0 — without /cmd_vel feedback
the planner has no way to know the robot isn't moving.

Conditions (both required):
  1. `|intent_v| > intent_motion_threshold` (planner asked for motion)
  2. `len(recent_cmd_v) >= n` AND `all(v <= cmdvel_motion_threshold)`

Pinned by these tests:
  (a) Both conditions met → wedged
  (b) Planner emits low intent (BLOCKED, classifier give-up) → not wedged
      (the planner is honestly stopped; recovery should not arm)
  (c) /cmd_vel has any motion → not wedged (robot is making progress)
  (d) Buffer not yet full → not wedged (just started, give it time)
 (e) Realistic scenario reproduces correctly
"""
from __future__ import annotations

from par_reactive_nav.nd_planner import is_wedged


# Defaults matching nd_planner ROS params
N = 20
INTENT_THRESHOLD = 0.03
CMDVEL_THRESHOLD = 0.02


def test_both_conditions_met_triggers_wedge():
    """Canonical wedge: planner LS1 v=0.07, /cmd_vel all zeros."""
    assert is_wedged(
        intent_v=0.07,
        recent_cmd_v=[0.0] * N,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_planner_giving_up_does_not_trigger_wedge():
    """If planner emits |v| <= intent_threshold (e.g. BLOCKED), it has acknowledged
    the situation. No wedge — recovery should not arm because the planner is
    honestly stopped, not overridden by safety."""
    assert not is_wedged(
        intent_v=0.0,
        recent_cmd_v=[0.0] * N,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )
    # Boundary case: exactly at threshold counts as "not wanting to move"
    assert not is_wedged(
        intent_v=INTENT_THRESHOLD,
        recent_cmd_v=[0.0] * N,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_robot_moving_does_not_trigger_wedge():
    """If /cmd_vel shows motion, robot is making progress. No wedge regardless of
    what the planner says."""
    moving = [0.05] * N  # robot actually moving at 5 cm/s
    assert not is_wedged(
        intent_v=0.07,
        recent_cmd_v=moving,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_buffer_not_full_does_not_trigger_wedge():
    """During mode-flip ramp-up, the cmd_vel buffer hasn't accumulated N samples
    yet. Don't false-fire while still warming up."""
    short_buffer = [0.0] * (N - 1)  # one sample short
    assert not is_wedged(
        intent_v=0.07,
        recent_cmd_v=short_buffer,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_one_motion_sample_in_buffer_breaks_wedge():
    """Even one /cmd_vel sample above threshold means the robot is moving — no wedge."""
    mostly_stopped = [0.0] * (N - 1) + [0.05]  # last one shows motion
    assert not is_wedged(
        intent_v=0.07,
        recent_cmd_v=mostly_stopped,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_negative_intent_v_still_counts_as_motion_intent():
    """Reverse intent (DEAD_END_LS2 v=-0.10) is still 'planner wants motion'."""
    assert is_wedged(
        intent_v=-0.10,
        recent_cmd_v=[0.0] * N,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_realistic_i50_scenario():
    """The exact case from session_20260510_1456: ND emits LS1 v=0.07 every
    100 ms, /cmd_vel reads 0 every tick (clamp=tof at the arbiter). After 20
    /cmd_vel ticks (~1.0 s of actual halt while planner ran ~10 ticks), wedge
    fires."""
    intent_v_ls1 = 0.07
    cmdvel_zeros_from_tof_clamp = [0.0] * N
    assert is_wedged(
        intent_v=intent_v_ls1,
        recent_cmd_v=cmdvel_zeros_from_tof_clamp,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )


def test_high_safety_cruise_in_open_arena_does_not_wedge():
    """HSGR cruise (planner v=0.18, /cmd_vel ~0.10 after H6 cap) should never wedge."""
    cruise_cmd_v = [0.10] * N
    assert not is_wedged(
        intent_v=0.18,
        recent_cmd_v=cruise_cmd_v,
        n=N,
        intent_motion_threshold=INTENT_THRESHOLD,
        cmdvel_motion_threshold=CMDVEL_THRESHOLD,
    )
