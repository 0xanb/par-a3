"""par_reactive_nav.vfh_planner — pick a heading from the polar histogram."""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray

from par_core.mode_filter import ModeState
from par_msgs.msg import ActiveMode, CommandIntent, TrialEvent

from .vfh_core import (
    VFHConfig,
    heading_to_yaw_rate,
    is_dead_end,
    select_heading,
    should_emit_stale_event,
)


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class VFHPlanner(Node):
    def __init__(self) -> None:
        super().__init__("vfh_planner")
        self.declare_parameter("hist_topic", "/par/polar_hist")
        self.declare_parameter("intent_topic", "/par/intents")
        self.declare_parameter("events_topic", "/par/events")
        # Cruise default 0.18 m/s. The earlier 0.12 made the robot
        # look stuck during demo-eve testing because in a tight room the
        # planner spends a lot of time in low-|w| AVOID mode where the
        # slowdown formula already cuts v hard. At 0.18 the LIDAR halo
        # (lidar_stop_m=0.15) still has a ~0.83 s reaction window before a
        # hard stop — comfortable for the 50 ms arbiter tick. Tunable via
        # ROS param if a smaller / larger arena needs a different cruise.
        self.declare_parameter("cruise_v", 0.18)
        self.declare_parameter("k_heading", 1.2)
        self.declare_parameter("n_bins", 72)
        # Make the geometry tunable without rebuilding. Demo arena ~0.5 m
        # clearance: thresholds of 0.45/0.05 mean a 0.50 m corridor passes.
        # Open warehouse: bump back up (e.g. 1.0/0.20).
        self.declare_parameter("obstacle_threshold_m", 0.45)
        self.declare_parameter("safety_margin_m", 0.05)
        self.declare_parameter("valley_min_width_bins", 2)
        self.declare_parameter("depth_bonus_weight", 8.0)
        self.declare_parameter("cone_bins_forward", 8)
        # Lowered 0.95 → 0.70 so DEAD_END fires when most of
        # the forward cone is blocked, even with one or two bins reading
        # marginally above threshold (a common pattern in corners where the
        # walls hit the cone edge at slight angles). Without this, the
        # planner kept picking near-forward bins with 1-2 cm gaps and the
        # halo clamped progress; recovery never armed because the strict
        # 0.95 threshold was rarely crossed.
        # Lowered 0.70 → 0.55 : declare DEAD_END earlier
        # so recovery starts BEFORE the chassis rides onto the obstacle
        # and tilts. The previous 0.70 fraction was tuned for the
        # high-confidence "no path forward" case; in tight dead-end
        # corners reactive would push forward into a 60-65% blocked
        # histogram, ride onto the lip, and trip the tilt FSM. Earlier
        # DEAD_END lets recovery_controller back out cleanly.
        self.declare_parameter("dead_end_blocked_frac", 0.55)
        # Chassis half-width for angular obstacle inflation (Borenstein &
        # Koren VFH+ original "dilation"). Set to 0 to disable. Match
        # perception_fusion's chassis_half_width_m for histogram parity.
        self.declare_parameter("chassis_half_width_m", 0.165)
        # Stuck-watchdog window: if every published intent in the last
        # `stuck_watchdog_n` ticks had |v| ≤ stuck_watchdog_v, force a
        # DEAD_END label regardless of histogram. Catches the "wedged but
        # planner thinks it found a valley" case where the chosen heading
        # is technically open in the histogram but the halo clamps motion
        # because the obstacle is in the actual forward physical cone.
        self.declare_parameter("stuck_watchdog_n", 20)
        self.declare_parameter("stuck_watchdog_v", 0.03)
        self.declare_parameter("stale_threshold_s", 0.5)
        self.declare_parameter("stale_debounce_s", 1.0)

        self._cfg = VFHConfig(
            n_bins=int(self.get_parameter("n_bins").value),
            obstacle_threshold_m=float(self.get_parameter("obstacle_threshold_m").value),
            safety_margin_m=float(self.get_parameter("safety_margin_m").value),
            valley_min_width_bins=int(self.get_parameter("valley_min_width_bins").value),
            depth_bonus_weight=float(self.get_parameter("depth_bonus_weight").value),
            cone_bins_forward=int(self.get_parameter("cone_bins_forward").value),
            dead_end_blocked_frac=float(self.get_parameter("dead_end_blocked_frac").value),
            chassis_half_width_m=float(self.get_parameter("chassis_half_width_m").value),
        )
        self._cruise = float(self.get_parameter("cruise_v").value)
        self._k = float(self.get_parameter("k_heading").value)
        self._previous_bin: int | None = None
        self._stuck_watchdog_n = int(self.get_parameter("stuck_watchdog_n").value)
        self._stuck_watchdog_v = float(self.get_parameter("stuck_watchdog_v").value)
        self._recent_v: list[float] = []   # ring of last published linear x
        self._stale_threshold_s = float(self.get_parameter("stale_threshold_s").value)
        self._stale_debounce_s = float(self.get_parameter("stale_debounce_s").value)

        # Prime the freshness clock at boot so a totally-silent perception_fusion
        # produces a stale event after stale_threshold_s rather than never. See
        # (eval-note expectation on stale sensors).
        self._last_hist_at: float = time.monotonic()
        self._last_stale_emit_at: float | None = None

        # : vfh_planner now gates on Mode "A" in the 2-mode
        # runtime, matching nd_planner. Previously gated on legacy "C" so it
        # stayed silent whenever launched via project_2mode.launch.py with
        # algo:=vfh_plus — blocking the entire algorithm-ablation arm of the
        # trial campaign. snapshot-pre-2mode rollback uses git tag.
        self._mode = ModeState("A", default_mode="IDLE")
        self.create_subscription(
            ActiveMode, "/par/active_mode",
            lambda m: self._mode.update(m.mode),
            _LATCHED_QOS,
        )

        self.create_subscription(
            Float32MultiArray, self.get_parameter("hist_topic").value, self._on_hist, 10,
        )
        self.pub = self.create_publisher(
            CommandIntent, self.get_parameter("intent_topic").value, 10,
        )
        self.events_pub = self.create_publisher(
            TrialEvent, self.get_parameter("events_topic").value, 10,
        )
        # 5 Hz watchdog tick — independent of /par/polar_hist arrival rate so
        # the planner can detect the topic going silent.
        self.create_timer(0.2, self._tick_stale_watchdog)
        self.get_logger().info("vfh_planner online")

    def _tick_stale_watchdog(self) -> None:
        # Stale watchdog only matters when our mode is the one that consumes
        # /par/polar_hist. In other modes, perception_fusion is silent by
        # design, so reporting the silence as a fault is misleading.
        if not self._mode.is_active():
            return
        now = time.monotonic()
        if not should_emit_stale_event(
            self._last_hist_at,
            now,
            stale_threshold_s=self._stale_threshold_s,
            last_emit_at=self._last_stale_emit_at,
            debounce_s=self._stale_debounce_s,
        ):
            return
        self._last_stale_emit_at = now
        ev = TrialEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.event = "stale_perception"
        ev.detail = (
            f"vfh_planner: no /par/polar_hist for "
            f"{now - self._last_hist_at:.2f}s "
            f"(threshold={self._stale_threshold_s:.2f}s)"
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

        out = CommandIntent()
        out.stamp = self.get_clock().now().to_msg()
        out.source = "reactive"
        out.priority = 70

        # Stuck-watchdog: if the planner has been publishing near-zero v
        # for the last N ticks, the robot is wedged regardless of what the
        # histogram says. Force a DEAD_END so the recovery FSM arms.
        wedged = (
            len(self._recent_v) >= self._stuck_watchdog_n
            and all(abs(v) <= self._stuck_watchdog_v for v in self._recent_v)
        )

        if is_dead_end(hist, self._cfg) or wedged:
            # Publish a controlled backup intent; recovery_controller will
            # take over with a higher-priority intent if needed.
            out.cmd.linear.x = -0.08
            out.cmd.angular.z = 0.8
            out.label = "DEAD_END"
            out.confidence = 0.9
            self.pub.publish(out)
            self._track_v(out.cmd.linear.x)
            return

        chosen = select_heading(hist, self._cfg, goal_bin=0,
                                previous_bin=self._previous_bin)
        if chosen is None:
            out.cmd.linear.x = 0.0
            out.cmd.angular.z = 0.0
            out.label = "BLOCKED"
            out.confidence = 1.0
        else:
            self._previous_bin = chosen
            w = heading_to_yaw_rate(chosen, self._cfg, k=self._k, w_max=1.0)
            # Slow down when turning hard.
            v = self._cruise * max(0.2, 1.0 - abs(w) / 1.5)
            out.cmd.linear.x = v
            out.cmd.angular.z = w
            out.label = "AVOID" if abs(w) > 0.3 else "FORWARD"
            out.confidence = 1.0
        self.pub.publish(out)
        self._track_v(out.cmd.linear.x)

    def _track_v(self, v: float) -> None:
        """Append the latest published linear velocity to the ring used by
        the stuck-watchdog. Trimmed to ``stuck_watchdog_n`` entries."""
        self._recent_v.append(float(v))
        if len(self._recent_v) > self._stuck_watchdog_n:
            self._recent_v = self._recent_v[-self._stuck_watchdog_n:]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VFHPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
