"""par_reactive_nav.nd_planner — Nearness Diagram reactive planner node.

Drop-in alternative to vfh_planner. Same `/par/polar_hist` subscription, same
`/par/intents` publication contract (source="reactive", priority 70). Post-
hybrid: emits `DEAD_END_LS2` (geometric dead-end) and `DEAD_END_WEDGE`
(planner-level stuck-watchdog) labels that the recovery_controller picks up via
its `label.startswith("DEAD_END")` trigger.

The classifier and per-state controllers live in `nd_core.py` (pure-function
module, no ROS imports). This wrapper is the boundary between ROS and the
core: it handles param resolution, mode gating, the polar-hist freshness
watchdog, and intent publishing.
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray

from par_core.mode_filter import ModeState
from par_msgs.msg import ActiveMode, CommandIntent, TrialEvent

from .nd_core import NDConfig, classify_and_command


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def is_wedged(
    *,
    intent_v: float,
    recent_cmd_v: list[float],
    n: int,
    intent_motion_threshold: float,
    cmdvel_motion_threshold: float,
) -> bool:
    """True when the planner wants to move but the safety layer is zeroing
 /cmd_vel. Detects the wedge: ND emits LS1 v=0.07 indefinitely while
    H1 ToF zeros both axes, and the robot makes no progress.

    Conditions (both required):
      1. ``|intent_v| > intent_motion_threshold`` — planner asked for motion.
      2. ``len(recent_cmd_v) >= n and all(v <= cmdvel_motion_threshold)``
         — actual /cmd_vel has been near-zero for n consecutive samples.

    Returns False if either condition is unmet (still ramping up post-mode-flip,
    or the planner itself emits near-zero so the decision is honest).
    """
    if abs(intent_v) <= intent_motion_threshold:
        return False
    if len(recent_cmd_v) < n:
        return False
    return all(v <= cmdvel_motion_threshold for v in recent_cmd_v)


class NDPlanner(Node):
    def __init__(self) -> None:
        super().__init__("nd_planner")
        self.declare_parameter("hist_topic", "/par/polar_hist")
        self.declare_parameter("intent_topic", "/par/intents")
        self.declare_parameter("events_topic", "/par/events")
        # evening: lowered from 0.18 → 0.12 for the demo arena.
        # Slower forward speed means impact jerk on a low obstacle is
        # smaller, the IMU detects it earlier, and recovery has more time
        # to react before the chassis is tilted. Trade-off: ~33 % slower
        # cruise during trials. Raise back via launch arg if needed.
        self.declare_parameter("cruise_v", 0.12)
        self.declare_parameter("n_bins", 72)
        self.declare_parameter("R_max_m", 5.0)
        self.declare_parameter("safety_dist_m", 0.30)
        self.declare_parameter("wide_threshold_bins", 10)
        self.declare_parameter("valley_min_width_bins", 2)
        self.declare_parameter("forward_cone_bins", 24)
        self.declare_parameter("backup_v", -0.10)
        self.declare_parameter("backup_w", 0.8)
        self.declare_parameter("chassis_half_width_m", 0.165)
        self.declare_parameter("stale_threshold_s", 0.5)
        self.declare_parameter("stale_debounce_s", 1.0)
        # Stuck-watchdog (post ): the planner alone cannot detect the
        # wedge — ND's LS1 emits v=0.07 (above any planner-side
        # threshold), but the H1 ToF halo zeros /cmd_vel to 0. So the
        # watchdog needs FEEDBACK: it tracks the actual /cmd_vel.linear.x
        # post-safety-clamp. Wedge fires when (a) the planner's last
        # decision wanted to move (|decision.v| > intent_motion_threshold)
        # AND (b) /cmd_vel has been near-zero (|v| <= cmdvel_motion_threshold)
        # for stuck_watchdog_n consecutive samples. This is the canonical
        # "intent vs. actual" mismatch pattern.
        self.declare_parameter("stuck_watchdog_n", 20)
        self.declare_parameter("intent_motion_threshold", 0.03)
        self.declare_parameter("cmdvel_motion_threshold", 0.02)

        self._cfg = NDConfig(
            n_bins=int(self.get_parameter("n_bins").value),
            R_max_m=float(self.get_parameter("R_max_m").value),
            safety_dist_m=float(self.get_parameter("safety_dist_m").value),
            wide_threshold_bins=int(self.get_parameter("wide_threshold_bins").value),
            valley_min_width_bins=int(self.get_parameter("valley_min_width_bins").value),
            cruise_v=float(self.get_parameter("cruise_v").value),
            forward_cone_bins=int(self.get_parameter("forward_cone_bins").value),
            backup_v=float(self.get_parameter("backup_v").value),
            backup_w=float(self.get_parameter("backup_w").value),
            chassis_half_width_m=float(self.get_parameter("chassis_half_width_m").value),
        )
        self._stale_threshold_s = float(self.get_parameter("stale_threshold_s").value)
        self._stale_debounce_s = float(self.get_parameter("stale_debounce_s").value)
        self._last_hist_at: float = time.monotonic()
        self._last_stale_emit_at: float | None = None

        self._stuck_watchdog_n = int(self.get_parameter("stuck_watchdog_n").value)
        self._intent_motion_threshold = float(
            self.get_parameter("intent_motion_threshold").value
        )
        self._cmdvel_motion_threshold = float(
            self.get_parameter("cmdvel_motion_threshold").value
        )
        # Once the wedge predicate trips, hold the assertion for this
        # long so the recovery_controller (trigger_hold_s=0.3) has time
        # to engage its 1.5 s reverse + 1.0 s spin phases. 0.6 s gives
        # comfortable headroom over the 0.3 s threshold while still
        # releasing quickly if the chassis is actually moving.
        self.declare_parameter("wedge_hold_s", 0.6)
        self._wedge_hold_s = float(self.get_parameter("wedge_hold_s").value)
        self._wedge_hold_until: float | None = None
        # DEAD_END_WEDGE intent linear velocity. Pure reverse (no spin) —
        # the recovery_controller SPIN phase owns rotation. -0.08 m/s is
        # gentle enough to avoid jerking the chassis off the obstacle but
        # fast enough to escape contact within the 0.6 s wedge-hold window.
        self.declare_parameter("wedge_reverse_v", -0.08)
        self._wedge_reverse_v = float(
            self.get_parameter("wedge_reverse_v").value
        )
        # Ring of last N |/cmd_vel.linear.x| values (post-safety-clamp).
        # Only sampled while the planner is asking for motion (watchdog armed).
        # The arm gate prevents cold-start cmd_v=0 history from the supervisor's
        # ANNOUNCE_360 spin from being interpreted as "robot wedged" the
        # instant nd_planner first publishes an HSGR cruise intent.
        self._recent_cmd_v: list[float] = []
        self._planner_intends_motion: bool = False
        self._arm_time: float | None = None

        # Mode gate (updated for 2-mode pivot post-): nd_planner is
        # the reactive backend for "Mode A" in the 2-mode demo. default_mode
        # ="IDLE" so the gate stays CLOSED until /par/active_mode = "A" lands;
        # without it, ModeState would self-activate and ND would emit
        # priority-70 intents during the supervisor's priority-60 360 announce.
        self._mode = ModeState("A", default_mode="IDLE")
        self.create_subscription(
            ActiveMode, "/par/active_mode",
            lambda m: self._mode.update(m.mode),
            _LATCHED_QOS,
        )

        self.create_subscription(
            Float32MultiArray, self.get_parameter("hist_topic").value, self._on_hist, 10,
        )
        # Watchdog feedback: post-safety-clamp /cmd_vel from arbiter.
        self.create_subscription(
            TwistStamped, "/cmd_vel", self._on_cmd_vel,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.pub = self.create_publisher(
            CommandIntent, self.get_parameter("intent_topic").value, 10,
        )
        self.events_pub = self.create_publisher(
            TrialEvent, self.get_parameter("events_topic").value, 10,
        )
        self.create_timer(0.2, self._tick_stale_watchdog)
        self.get_logger().info("nd_planner online")

    def _tick_stale_watchdog(self) -> None:
        if not self._mode.is_active():
            return
        now = time.monotonic()
        elapsed = now - self._last_hist_at
        if elapsed < self._stale_threshold_s:
            return
        if self._last_stale_emit_at is not None and (
            now - self._last_stale_emit_at < self._stale_debounce_s
        ):
            return
        self._last_stale_emit_at = now
        ev = TrialEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.event = "stale_perception"
        ev.detail = (
            f"nd_planner: no /par/polar_hist for "
            f"{elapsed:.2f}s (threshold={self._stale_threshold_s:.2f}s)"
        )
        self.events_pub.publish(ev)
        self.get_logger().warn(ev.detail)

    def _on_hist(self, msg: Float32MultiArray) -> None:
        self._last_hist_at = time.monotonic()
        if not self._mode.is_active():
            return
        hist = np.array(msg.data, dtype=np.float32)
        # Re-inflate 1e6 placeholders back to +inf for the core.
        hist = np.where(hist >= 1e6 - 1, np.inf, hist)

        decision = classify_and_command(hist, self._cfg)

        # Stuck-watchdog (post ): detect intent-vs-actual mismatch.
        # The planner's intent says "go" (LS1 v=0.07, HSGR v=0.18, etc.) but
        # /cmd_vel has been zeroed by the safety layer (H1 ToF halo zeros
        # both axes when front ToF reads close). Without /cmd_vel feedback,
        # nd_planner cannot distinguish "robot is moving as commanded" from
        # "robot is wedged with safety override". Compare the planner's last
        # commanded v against the actual /cmd_vel ring buffer.
        #
        # Arming gate: only sample /cmd_vel while the planner is actively
        # asking for motion, AND only fire wedged once the ring buffer has
        # filled with samples observed AFTER arming. Without this, the
        # supervisor's 21 s ANNOUNCE_360 spin (cmd_v=0, w=0.30) supplies a
        # full window of cmd_v=0 history before nd_planner ever publishes,
        # causing a false-positive wedge on the very first HSGR intent.
        intent_wants_motion = abs(float(decision.v)) > self._intent_motion_threshold
        if intent_wants_motion and not self._planner_intends_motion:
            self._recent_cmd_v = []
            self._arm_time = time.monotonic()
        elif not intent_wants_motion:
            self._arm_time = None
            self._recent_cmd_v = []
        self._planner_intends_motion = intent_wants_motion

        wedged_now = False
        if (
            self._arm_time is not None
            and len(self._recent_cmd_v) >= self._stuck_watchdog_n
        ):
            wedged_now = is_wedged(
                intent_v=float(decision.v),
                recent_cmd_v=self._recent_cmd_v,
                n=self._stuck_watchdog_n,
                intent_motion_threshold=self._intent_motion_threshold,
                cmdvel_motion_threshold=self._cmdvel_motion_threshold,
            )

        # Hold the wedge assertion for at least ``_wedge_hold_s`` once
        # tripped so the recovery_controller (which requires
        # ``trigger_hold_s=0.3`` of sustained DEAD_END to engage its
        # reverse phase) has time to take over. Without the hold the
        # watchdog flickers between HSGR and DEAD_END_WEDGE every
        # tick: the brief reverse motion drops a non-zero sample into
        # the ring, is_wedged returns False, planner reverts to HSGR,
        # ToF zeros forward, ring fills with zeros, watchdog fires
        # again — oscillation with no actual escape. (Diagnosed
        # evening with the slipper-in-front session.)
        now_t = time.monotonic()
        if wedged_now:
            self._wedge_hold_until = now_t + self._wedge_hold_s
        wedged = (
            self._wedge_hold_until is not None and now_t < self._wedge_hold_until
        )
        if not wedged:
            self._wedge_hold_until = None

        out = CommandIntent()
        out.stamp = self.get_clock().now().to_msg()
        out.source = "reactive"
        out.priority = 70

        if wedged:
            # Pure reverse — the recovery_controller's SPIN phase handles
            # rotation. A combined back-and-pivot here used to be perceived
            # by the operator as "U-turn instead of back up" because the
            # angular component leaked across the brief reverse phase via
            # H5 accel limiter momentum, then ran into the SPIN phase.
            out.cmd.linear.x = self._wedge_reverse_v
            out.cmd.angular.z = 0.0
            out.label = "DEAD_END_WEDGE"
            out.confidence = 0.9
        else:
            out.cmd.linear.x = float(decision.v)
            out.cmd.angular.z = float(decision.w)
            out.label = decision.label
            out.confidence = float(decision.confidence)
        self.pub.publish(out)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        """Track the actual post-safety-clamp /cmd_vel for the stuck-watchdog.

        Only sample while the planner is asking for motion (watchdog armed).
        Cold-start cmd_v=0 samples from the supervisor announce phase or
        any AVOID interval must not pollute the wedge-detection window.
        """
        if self._arm_time is None:
            return
        self._recent_cmd_v.append(abs(float(msg.twist.linear.x)))
        if len(self._recent_cmd_v) > self._stuck_watchdog_n:
            self._recent_cmd_v = self._recent_cmd_v[-self._stuck_watchdog_n:]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NDPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
