"""par_anomaly.detectors — pure-function anomaly predicates.

No ROS imports. Every predicate is independently unit-testable under
``colcon test`` without rclpy / sensor_msgs. The ROS wrapper in
``anomaly_detector.py`` is a thin glue layer that holds the rolling state
and calls these predicates on each callback.

Three detectors live here:

* ``is_tilted`` + ``quat_to_roll_pitch`` — chassis orientation watchdog
* ``is_collision_impact`` — IMU jerk spike detector
* ``is_stalled`` — cmd_vel vs filtered-odom divergence

Cold-start arm-gate notes follow the pattern documented in
``nd_planner.is_wedged``. The stall detector requires the same
gate; the wrapper in ``anomaly_detector.py`` owns the arm state.
"""
from __future__ import annotations

import math


# ---------------------------------------------------------------------------
# Tilt detector (Phase 2)
# ---------------------------------------------------------------------------


def quat_to_roll_pitch(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float]:
 """Convert a unit quaternion to ZYX Tait-Bryan roll + pitch (radians).

 Yaw is intentionally ignored — tilt detection only cares about
 deviation from the level horizontal plane. The convention matches
 REP-103 (x forward, y left, z up) so a chassis nose-up reads
 ``pitch > 0`` and a chassis right-side-up-but-leaning-left reads
 ``roll > 0``.

 Includes the standard gimbal-lock guard at ``|sin(pitch)| ≥ 1``.
 """
 sin_p = 2.0 * (qw * qy - qz * qx)
 if abs(sin_p) >= 1.0:
 pitch = math.copysign(math.pi / 2.0, sin_p)
 else:
 pitch = math.asin(sin_p)
 sin_r_cos_p = 2.0 * (qw * qx + qy * qz)
 cos_r_cos_p = 1.0 - 2.0 * (qx * qx + qy * qy)
 roll = math.atan2(sin_r_cos_p, cos_r_cos_p)
 return roll, pitch


def is_tilted(
 *,
 roll_rad: float,
 pitch_rad: float,
 max_roll_rad: float = math.radians(8.0),
 max_pitch_rad: float = math.radians(8.0),
) -> bool:
 """True when the chassis is tilted beyond the level-floor envelope.

 Default thresholds (8°) are tuned for the ROSbot 3 PRO on indoor
 tile / hardwood. Drift on a level floor is < 1°; an 8° pitch /
 roll corresponds to one wheel being ~5 cm higher than the other
 (chassis half-width 0.165 m → tan(8°) × 0.165 ≈ 23 mm of vertical
 deflection across the wheelbase; in practice this is "noticeably
 tilted, time to halt"). Tunable as ROS params for floors with
 higher noise.
 """
 return abs(roll_rad) > max_roll_rad or abs(pitch_rad) > max_pitch_rad


def is_level(
 *,
 roll_rad: float,
 pitch_rad: float,
 clear_roll_rad: float = math.radians(5.0),
 clear_pitch_rad: float = math.radians(5.0),
) -> bool:
 """True when the chassis is well within the level envelope.

 Hysteresis partner to ``is_tilted``: once tripped, the detector
 must observe ``is_level`` continuously for a release window
 (default 1.0 s in the wrapper) before clearing the TILT_STOP
 intent. The 5° clear band sits below the 8° trip band so a
 chassis hovering at the threshold does not oscillate.
 """
 return abs(roll_rad) < clear_roll_rad and abs(pitch_rad) < clear_pitch_rad


# ---------------------------------------------------------------------------
# Collision impact detector (Phase 3)
# ---------------------------------------------------------------------------


def is_collision_impact(
 recent_accel_x: list[float],
 *,
 jerk_threshold_m_s3: float = 8.0,
 sample_dt_s: float = 0.01,
 window_n: int = 5,
) -> bool:
 """True when chassis x-axis jerk exceeds the threshold in a short window.

 Jerk is approximated as the largest first difference of the linear
 acceleration ring divided by the sample period. A real collision
 produces a deceleration spike — the chassis stops abruptly when it
 hits an obstacle the planner did not avoid. We look at the last
 ``window_n`` samples (default 50 ms at 100 Hz from the BNO055) so a
 transient spike rather than a sustained drift trips the predicate.

 Used in combination with ``is_stalled``: a real collision both
 spikes the IMU and stops the wheels. Operator-conditioning rules
 out rumble strips and elevator gaps, which jerk without arresting
 chassis motion.

 The wrapper owns the rolling buffer; this function is pure.
 """
 if len(recent_accel_x) < window_n:
 return False
 seg = recent_accel_x[-window_n:]
 max_delta = 0.0
 for i in range(len(seg) - 1):
 d = abs(seg[i + 1] - seg[i]) / sample_dt_s
 if d > max_delta:
 max_delta = d
 return max_delta > jerk_threshold_m_s3


# ---------------------------------------------------------------------------
# Wheel-stall detector (Phase 3)
# ---------------------------------------------------------------------------


def is_stalled(
 *,
 recent_cmd_v: list[float],
 recent_odom_v: list[float],
 n: int = 20,
 cmd_threshold: float = 0.05,
 odom_threshold: float = 0.02,
) -> bool:
 """True when /cmd_vel commanded motion but the chassis did not deliver.

 Structurally identical to ``nd_planner.is_wedged`` but uses the
 EKF-filtered chassis velocity from ``/odometry/filtered`` instead
 of post-clamp ``/cmd_vel``. Catches the contact case where the
 safety layer let the command through and the wheels still failed
 to translate the chassis (motor stall, mechanical bind, slipping
 wheels on a polished floor, low obstacle holding the chassis
 against a wall).

 Cold-start arm gate (pattern): the wrapper must clear both
 rings on the rising edge of ``cmd_v > cmd_threshold`` and only
 sample odom while armed. Otherwise pre-arm ``odom_v=0`` history
 poisons the wedge signal — the same false-positive that documented for nd_planner before landed.

 Both averages are arithmetic means over the trailing ``n`` samples.
 The function is pure: pass full, post-arm buffers in.
 """
 if len(recent_cmd_v) < n or len(recent_odom_v) < n:
 return False
 cmd_avg = sum(recent_cmd_v[-n:]) / n
 odom_avg = sum(recent_odom_v[-n:]) / n
 return cmd_avg > cmd_threshold and odom_avg < odom_threshold


def is_wheel_impact(
 *,
 recent_cmd_v: list[float],
 recent_odom_v: list[float],
 n: int = 6,
 cmd_threshold: float = 0.03,
 odom_threshold: float = 0.02,
) -> bool:
 """Short-window stall — catches transient impact within ~300 ms.

 Same shape as ``is_stalled`` but with a shorter rolling window so a
 brief impact (chassis hits a low obstacle, wheels keep spinning but
 chassis decelerates) registers before the 1 s ``is_stalled`` window
 has had time to average. The session showed that
 a slow-cruise (0.057 m/s) impact tilted the chassis BEFORE either
 the 8 m/s³ jerk threshold or the 1 s stall window could fire.

 ``recent_odom_v`` should be EKF-fused chassis velocity from
 ``/odometry/filtered.twist.twist.linear.x``. Raw wheel encoder
 velocity would NOT work here — when wheels slip against a wedging
 obstacle they keep spinning while the chassis is pinned. The EKF
 fuses IMU + encoders; under contact the IMU dominates and the
 filter reports the correct (zero) chassis velocity within ~150 ms.

 Wrapper must apply the cold-start arm gate (same as
 ``is_stalled``) so pre-arm zeros do not poison the buffer.
 """
 if len(recent_cmd_v) < n or len(recent_odom_v) < n:
 return False
 cmd_avg = sum(recent_cmd_v[-n:]) / n
 odom_avg = sum(recent_odom_v[-n:]) / n
 return cmd_avg > cmd_threshold and odom_avg < odom_threshold
