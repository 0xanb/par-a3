"""Unit tests for par_core.SafetyLayer — run via ``colcon test``.

Each test isolates one of the seven kill paths so a regression has a clear
owner. No ROS, no sensors, no threading.
"""
from __future__ import annotations

from geometry_msgs.msg import Twist

from par_core import SafetyConfig, SafetyLayer


def mk_cmd(v: float = 0.2, w: float = 0.0) -> Twist:
    t = Twist()
    t.linear.x = v
    t.angular.z = w
    return t


def test_deadman_zero_when_disarmed() -> None:
    sl = SafetyLayer(SafetyConfig())
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=None, lidar_front_min_m=None,
                           cmd_stamp_s=0.0, armed=False, now_s=0.0)
    assert reason == "deadman"
    assert out.linear.x == 0.0
    assert out.angular.z == 0.0


def test_tof_hardware_zero_when_close() -> None:
    sl = SafetyLayer(SafetyConfig())
    # prime the watchdog so H3 does not fire on the real call
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=0.05, lidar_front_min_m=1.0,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == "tof"
    assert out.linear.x == 0.0


def test_watchdog_zero_on_gap() -> None:
    sl = SafetyLayer(SafetyConfig(watchdog_s=0.1))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=1.0, lidar_front_min_m=1.0,
                           cmd_stamp_s=0.5, armed=True, now_s=0.5)
    assert reason == "watchdog"
    assert out.linear.x == 0.0


def test_stale_command_zero() -> None:
    sl = SafetyLayer(SafetyConfig(stale_cmd_s=0.2, watchdog_s=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=1.0, lidar_front_min_m=1.0,
                           cmd_stamp_s=0.0, armed=True, now_s=1.0)
    assert reason == "stale"
    assert out.linear.x == 0.0


def test_lidar_halo_hard_stop() -> None:
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=1.0, lidar_front_min_m=0.15,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == "lidar_stop"
    assert out.linear.x <= 0.0


def test_lidar_halo_soft_scale() -> None:
    cfg = SafetyConfig(watchdog_s=10.0, lidar_slow_m=0.5, lidar_stop_m=0.25,
                       lin_accel_max=10.0)
    sl = SafetyLayer(cfg)
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=1.0, lidar_front_min_m=0.375,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == "lidar_slow"
    assert 0.0 < out.linear.x < 0.3


def test_speed_cap() -> None:
    sl = SafetyLayer(SafetyConfig(v_max=0.1, w_max=0.5, watchdog_s=10.0,
                                   lin_accel_max=10.0, ang_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, _ = sl.clamp(mk_cmd(5.0, 5.0), tof_m=1.0, lidar_front_min_m=1.0,
                      cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert out.linear.x <= 0.1 + 1e-9
    assert out.angular.z <= 0.5 + 1e-9


def test_accel_limit() -> None:
    sl = SafetyLayer(SafetyConfig(lin_accel_max=0.5, watchdog_s=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    # dt = 0.1 s, so max Δv = 0.05
    out, _ = sl.clamp(mk_cmd(1.0), tof_m=1.0, lidar_front_min_m=1.0,
                      cmd_stamp_s=0.1, armed=True, now_s=0.1)
    assert abs(out.linear.x - 0.05) < 1e-6


# ---- H2r LIDAR rear halo (audit edge case 3.5) -------------------------

def test_lidar_rear_halo_blocks_reverse() -> None:
    """Reverse intent must be zeroed when the rear halo trips. The forward
    cone is clear so the original H2 path stays silent.

    Default rear stop threshold is 0.10 m (tighter than forward 0.25 m
    because rear motion is slower). 0.05 m is well inside the stop band.
    """
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(-0.2), tof_m=1.0, lidar_front_min_m=1.0,
                           lidar_rear_min_m=0.05, cmd_stamp_s=0.01,
                           armed=True, now_s=0.01)
    assert reason == "lidar_rear_stop"
    assert out.linear.x >= 0.0  # reverse zeroed (or clamped to 0)


def test_lidar_rear_halo_ignored_when_moving_forward() -> None:
    """A close rear obstacle must NOT clamp a positive linear request — the
    rear halo only consults when v < 0."""
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.2), tof_m=1.0, lidar_front_min_m=1.0,
                           lidar_rear_min_m=0.05, cmd_stamp_s=0.01,
                           armed=True, now_s=0.01)
    assert reason == ""
    assert out.linear.x > 0.0


def test_lidar_rear_halo_soft_scale_on_reverse() -> None:
    """Between rear stop and rear slow distances, reverse velocity is scaled.

    Default rear thresholds: stop=0.10 m, slow=0.25 m. So 0.175 m sits
    midway and should scale reverse to half magnitude (before H5 accel
    smoothing).
    """
    cfg = SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0)
    sl = SafetyLayer(cfg)
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(-0.3), tof_m=1.0, lidar_front_min_m=1.0,
                           lidar_rear_min_m=0.175, cmd_stamp_s=0.01,
                           armed=True, now_s=0.01)
    assert reason == "lidar_rear_slow"
    # Reverse intent scaled toward zero (less negative) but not zeroed.
    assert -0.3 < out.linear.x < 0.0


def test_lidar_rear_halo_uses_rear_specific_thresholds() -> None:
    """The rear halo MUST consult lidar_rear_stop_m, not lidar_stop_m.

    With forward stop=0.25 and rear stop=0.10 (defaults), an obstacle
    at 0.15 m behind is past the rear stop band so reverse should NOT
    be hard-clamped, just scaled. Pre-fix this would have triggered
    'lidar_rear_stop' because the rear halo reused the forward 0.25 m
    threshold — that was the deadlock blocking TILT_REVERSE.
    """
    cfg = SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0)
    sl = SafetyLayer(cfg)
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(-0.05), tof_m=1.0, lidar_front_min_m=1.0,
                           lidar_rear_min_m=0.15, cmd_stamp_s=0.01,
                           armed=True, now_s=0.01)
    assert reason == "lidar_rear_slow"
    assert out.linear.x < 0.0  # reverse motion preserved (scaled)


# ---- H1 directional ToF (post- chassis-touch escape) --------------

def test_tof_front_permits_reverse_and_spin() -> None:
    """Front ToF tripping must NOT block reverse or angular spin. The
    recovery FSM relies on this to back the chassis out of touch range
    when a low obstacle below the LIDAR plane has pinned a corner ToF."""
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0,
                                   ang_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    # Reverse with front ToF close — should pass through.
    out_rev, reason_rev = sl.clamp(mk_cmd(-0.1), tof_m=0.05,
                                    lidar_front_min_m=1.0,
                                    cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason_rev == ""
    assert out_rev.linear.x < 0.0
    # Spin in place with front ToF close — should pass through.
    out_spin, reason_spin = sl.clamp(mk_cmd(0.0, 0.8), tof_m=0.05,
                                      lidar_front_min_m=1.0,
                                      cmd_stamp_s=0.02, armed=True, now_s=0.02)
    assert reason_spin == ""
    assert out_spin.angular.z > 0.0


def test_tof_front_still_blocks_forward() -> None:
    """A forward request with the front ToF tripping must still be zeroed."""
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(0.3), tof_m=0.05, lidar_front_min_m=1.0,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == "tof"
    assert out.linear.x <= 0.0


def test_tof_rear_blocks_reverse_when_provided() -> None:
    """Symmetric protection: rear ToF tripping must zero reverse intent."""
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(-0.2), tof_m=1.0, lidar_front_min_m=1.0,
                           tof_rear_m=0.05,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == "tof_rear"
    assert out.linear.x >= 0.0


def test_lidar_rear_halo_none_means_no_check() -> None:
    """Backward-compatible: if the caller omits lidar_rear_min_m, rear halo
    is silent (preserves pre-H2r behaviour for callers not yet wired)."""
    sl = SafetyLayer(SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0))
    sl.clamp(mk_cmd(0.0), tof_m=1.0, lidar_front_min_m=1.0,
             cmd_stamp_s=0.0, armed=True, now_s=0.0)
    out, reason = sl.clamp(mk_cmd(-0.2), tof_m=1.0, lidar_front_min_m=1.0,
                           cmd_stamp_s=0.01, armed=True, now_s=0.01)
    assert reason == ""
    assert out.linear.x < 0.0  # reverse allowed without rear data
