"""par_arbiter.arbiter — priority fusion of CommandIntents onto /cmd_vel.

Subscribes: /par/intents     par_msgs/CommandIntent         (many publishers)
            /scan            sensor_msgs/LaserScan          (H2 LIDAR halo)
            /range/fl        sensor_msgs/LaserScan          (H1 ToF front-left)
            /range/fr        sensor_msgs/LaserScan          (H1 ToF front-right)
            /par/deadman     std_msgs/Empty                 (H7 heartbeat)
Publishes:  /cmd_vel         geometry_msgs/TwistStamped     (20 Hz)
            /par/events      par_msgs/TrialEvent            (who won, what clamped)

Safety layer integration (resolved)
-----------------------------------------------
Each tick: resolve the winning CommandIntent, then hand it to
``par_core.SafetyLayer.clamp()`` which enforces:

    H1 ToF hardware     /range/{fl,fr} < tof_min_m -> zero
    H2 LIDAR halo       forward ±30° cone < lidar_stop_m -> zero
                        soft-slow between lidar_slow_m and lidar_stop_m
    H3 Watchdog         arbiter tick dropped > watchdog_s -> zero
    H4 Stale command    (also enforced by the resolve step's stale_after_s)
    H5 Accel limiter    clamp Δv/Δω per tick
    H6 Speed cap        v_max, w_max (per-tier from launch args)
    H7 Deadman          armed only when /par/deadman is fresh; or always-armed
                        when ``require_deadman:=False`` (default True for now,
                        so hardware runs without a wired gamepad heartbeat).

The physical e-stop sits above all seven.

Parameters
----------
rate_hz : float                 default 20.0
stale_after_s : float           default 0.5
grace_s : float                 default 0.1
decay_rate_pts_per_sec : float  default 20.0
v_max : float                   default 0.40   (H6 linear cap, per tier)
w_max : float                   default 1.50   (H6 angular cap, per tier)
use_safety_layer : bool         default True   — gate the full SafetyLayer
require_deadman : bool          default False  — when True, H7 zeroes if
                                                  /par/deadman is stale
deadman_fresh_s : float         default 0.3
tof_min_m : float               default 0.15   (H1)
lidar_slow_m : float            default 0.50   (H2)
lidar_stop_m : float            default 0.25   (H2)
lidar_front_cone_deg : float    default 30.0   (H2)
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

from par_core import SafetyConfig, SafetyLayer
from par_msgs.msg import CommandIntent, TrialEvent


def _laserscan_min_range(msg: LaserScan) -> float:
    lo = float(msg.range_min) if math.isfinite(msg.range_min) else 0.0
    hi = float(msg.range_max) if math.isfinite(msg.range_max) else float("inf")
    valid = [float(r) for r in msg.ranges if math.isfinite(r) and lo <= r <= hi]
    return min(valid) if valid else float("nan")

from .arbiter_core import ArbiterConfig, ScoredIntent, resolve


class Arbiter(Node):
    def __init__(self) -> None:
        super().__init__("arbiter")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("stale_after_s", 0.5)
        self.declare_parameter("grace_s", 0.1)
        self.declare_parameter("decay_rate_pts_per_sec", 20.0)
        self.declare_parameter("v_max", 0.40)
        self.declare_parameter("w_max", 1.50)
        self.declare_parameter("use_safety_layer", True)
        self.declare_parameter("require_deadman", False)
        self.declare_parameter("deadman_fresh_s", 0.3)
        self.declare_parameter("tof_min_m", 0.12)
        # LIDAR halo defaults tuned for SMALL DEMO ROOMS (≤ 4 m). Earlier
        # values (slow=0.50, stop=0.25) were inherited from open-warehouse
        # tutorials; in a 3 m demo arena they would hold the robot stationary
        # because chairs and operators were always within 0.5 m. Tightening
        # to slow=0.35 / stop=0.18 keeps the safety guarantee (>15 cm
        # standoff to obstacles) while letting the robot move at all.
        # "small-room LIDAR halo".
        self.declare_parameter("lidar_slow_m", 0.40)
        self.declare_parameter("lidar_stop_m", 0.20)
        # Rear halo defaults are tighter than forward — slow reverse can
        # creep closer to obstacles than fast forward. Without the split,
        # sandwich geometry (front + rear obstacles both within 0.20 m)
        # deadlocked the tilt FSM ( evening). See I-NN / M-NN.
        self.declare_parameter("lidar_rear_slow_m", 0.25)
        self.declare_parameter("lidar_rear_stop_m", 0.10)
        self.declare_parameter("lidar_front_cone_deg", 30.0)
        # Husarion ROSbot 3 PRO mounts the LIDAR with 180° yaw relative to
        # base_link. Without this offset the H2 forward-cone halo guards
        # the chassis-rear instead of chassis-forward, letting the robot
        # drive into walls in front while halting on rear obstacles. Match
        # this to perception_fusion's lidar_yaw_offset_rad. Per-deployment
        # overridable via ROS param; 0 for chassis-aligned LIDAR mounts.
        self.declare_parameter("lidar_yaw_offset_rad", math.pi)
        # Bench-test knob: disable proximity halos (H1 ToF + H2 LIDAR) so the
        # operator can verify intent → cmd_vel chains on a desk or with the
        # robot on blocks, where the room geometry would otherwise trip the
        # halos. Keeps H3 watchdog, H4 stale, H5 accel, H6 v_max, H7 deadman
        # active. Default False so cautious / normal / demo tiers stay safe.
        self.declare_parameter("disable_proximity_halos", False)

        rate = float(self.get_parameter("rate_hz").value)
        self._cfg = ArbiterConfig(
            stale_after_s=float(self.get_parameter("stale_after_s").value),
            grace_s=float(self.get_parameter("grace_s").value),
            decay_rate=float(self.get_parameter("decay_rate_pts_per_sec").value),
        )
        self._v_max = float(self.get_parameter("v_max").value)
        self._w_max = float(self.get_parameter("w_max").value)
        self._use_safety = bool(self.get_parameter("use_safety_layer").value)
        self._require_deadman = bool(self.get_parameter("require_deadman").value)
        self._deadman_fresh_s = float(self.get_parameter("deadman_fresh_s").value)
        self._lidar_front_cone_rad = math.radians(
            float(self.get_parameter("lidar_front_cone_deg").value)
        )
        self._lidar_yaw_offset_rad = float(
            self.get_parameter("lidar_yaw_offset_rad").value
        )

        self._disable_proximity_halos = bool(
            self.get_parameter("disable_proximity_halos").value
        )
        # When the bench-test flag is set, zero out the spatial-halo thresholds
        # so the H1 ToF and H2 LIDAR checks in SafetyLayer.clamp() never fire.
        # The other kill paths (H3, H4, H5, H6, H7) remain active.
        if self._disable_proximity_halos:
            tof_min = 0.0
            lidar_slow = 0.0
            lidar_stop = 0.0
            self.get_logger().warn(
                "disable_proximity_halos=True → H1 ToF + H2 LIDAR halos disabled "
                "(bench/desk test only; do NOT use on the floor)"
            )
        else:
            tof_min = float(self.get_parameter("tof_min_m").value)
            lidar_slow = float(self.get_parameter("lidar_slow_m").value)
            lidar_stop = float(self.get_parameter("lidar_stop_m").value)

        lidar_rear_slow = float(self.get_parameter("lidar_rear_slow_m").value)
        lidar_rear_stop = float(self.get_parameter("lidar_rear_stop_m").value)
        if self._disable_proximity_halos:
            lidar_rear_slow = 0.0
            lidar_rear_stop = 0.0

        self._safety = SafetyLayer(SafetyConfig(
            v_max=self._v_max,
            w_max=self._w_max,
            tof_min_m=tof_min,
            lidar_slow_m=lidar_slow,
            lidar_stop_m=lidar_stop,
            lidar_rear_slow_m=lidar_rear_slow,
            lidar_rear_stop_m=lidar_rear_stop,
        ))

        # Sensor state, updated by subscriptions.
        self._tof_fl: float = float("nan")
        self._tof_fr: float = float("nan")
        # H1r rear ToF pair. Hardware was unwired until :
        # SafetyLayer.clamp() accepted tof_rear_m but the arbiter never
        # subscribed to /range/rl + /range/rr, so the kwarg defaulted to None
        # and the rear-halo branch was dead code at runtime. audit found
        # five recovery publishers (TILT_REVERSE, RECOVER_REVERSE, VFH/ND
        # dead-end backups, stall recovery) silently relying on LIDAR-only
        # rear coverage that has documented low-obstacle blind spots.
        self._tof_rl: float = float("nan")
        self._tof_rr: float = float("nan")
        self._lidar_front_min: float = float("nan")
        # H2r rear halo: same 5th-percentile filter as the forward cone, but
        # over the rear ±cone. Consulted by SafetyLayer.clamp() only when the
        # winning intent has linear.x < 0 (recovery reverse, manual back-up).
        self._lidar_rear_min: float = float("nan")
        self._last_deadman_t: float = 0.0

        self._latest_by_source: dict[str, tuple[float, CommandIntent]] = {}
        self._last_winner: str | None = None

        self.sub_intents = self.create_subscription(
            CommandIntent, "/par/intents", self._on_intent, 20,
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sub_fl = self.create_subscription(
            LaserScan, "/range/fl", self._on_tof_fl,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sub_fr = self.create_subscription(
            LaserScan, "/range/fr", self._on_tof_fr,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sub_rl = self.create_subscription(
            LaserScan, "/range/rl", self._on_tof_rl,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sub_rr = self.create_subscription(
            LaserScan, "/range/rr", self._on_tof_rr,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sub_deadman = self.create_subscription(
            Empty, "/par/deadman", self._on_deadman, 10,
        )

        self.pub_cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.pub_ev = self.create_publisher(TrialEvent, "/par/events", 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"arbiter online: rate={rate} Hz, v_max={self._v_max}, w_max={self._w_max}, "
            f"safety_layer={self._use_safety}, require_deadman={self._require_deadman}"
        )

    # ---- sensor callbacks ---------------------------------------------------

    def _on_intent(self, msg: CommandIntent) -> None:
        self._latest_by_source[msg.source] = (time.monotonic(), msg)

    def _on_tof_fl(self, msg: LaserScan) -> None:
        # Husarion firmware "range_laserscan_fix" publishes each VL53L0X ToF as
        # a LaserScan with a narrow ±0.13 rad cone. Take the closest valid beam
        # so H1 trips on the worst-case obstacle in the cone.
        self._tof_fl = _laserscan_min_range(msg)

    def _on_tof_fr(self, msg: LaserScan) -> None:
        self._tof_fr = _laserscan_min_range(msg)

    def _on_tof_rl(self, msg: LaserScan) -> None:
        self._tof_rl = _laserscan_min_range(msg)

    def _on_tof_rr(self, msg: LaserScan) -> None:
        self._tof_rr = _laserscan_min_range(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        """5th-percentile range inside the forward ±cone (F7 polish).

        Plain min-of-cone is fragile: a single dropped or zero LIDAR beam
        can fire a false hard stop at H2. Sorting the cone's valid ranges
        and taking the 5th percentile preserves the worst-case behaviour
        when the cone is genuinely blocked, but absorbs single-beam outliers
        without drama.

        Also computes the rear-cone equivalent in a single pass for the
        H2r reverse-direction halo. The rear cone is centred on a = ±π in
        the (yaw-offset-applied) frame.
        """
        if not msg.ranges:
            self._lidar_front_min = float("nan")
            self._lidar_rear_min = float("nan")
            return
        cone = self._lidar_front_cone_rad
        front: list[float] = []
        rear: list[float] = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = msg.angle_min + i * msg.angle_increment + self._lidar_yaw_offset_rad
            # normalise to [-pi, pi]
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            if abs(a) <= cone:
                front.append(r)
            # Rear cone wraps around ±π. abs(|a| - π) ≤ cone captures the
            # ±cone-rad band centred on the chassis-rear direction.
            if abs(abs(a) - math.pi) <= cone:
                rear.append(r)

        def _percentile_5(vals: list[float]) -> float:
            if not vals:
                return float("nan")
            vals.sort()
            idx = max(0, int(0.05 * len(vals)) - 1) if len(vals) >= 20 else min(1, len(vals) - 1)
            return vals[idx]

        self._lidar_front_min = _percentile_5(front)
        self._lidar_rear_min = _percentile_5(rear)

    def _on_deadman(self, _msg: Empty) -> None:
        self._last_deadman_t = time.monotonic()

    # ---- tick ---------------------------------------------------------------

    def _tof_min(self) -> float:
        """Minimum of fl/fr ToF readings, ignoring NaN."""
        vals = [v for v in (self._tof_fl, self._tof_fr) if math.isfinite(v)]
        return min(vals) if vals else float("nan")

    def _tof_rear_min(self) -> float:
        """Minimum of rl/rr ToF readings, ignoring NaN.

 : consulted by SafetyLayer.clamp only when the
        winning intent has linear.x < 0 (any reverse recovery). NaN = no
        rear reading available; SafetyLayer treats NaN as "no data" and
        falls back to LIDAR-only rear coverage.
        """
        vals = [v for v in (self._tof_rl, self._tof_rr) if math.isfinite(v)]
        return min(vals) if vals else float("nan")

    def _armed(self, now: float) -> bool:
        if not self._require_deadman:
            return True
        return (now - self._last_deadman_t) < self._deadman_fresh_s

    def _tick(self) -> None:
        now = time.monotonic()
        candidates: list[ScoredIntent] = []
        latest_stamp_s: float | None = None
        for src, (t_received, msg) in self._latest_by_source.items():
            candidates.append(ScoredIntent(
                age_s=now - t_received,
                priority=int(msg.priority),
                confidence=float(msg.confidence),
                linear_x=float(msg.cmd.linear.x),
                angular_z=float(msg.cmd.angular.z),
                source=src,
                label=str(msg.label),
            ))
            latest_stamp_s = t_received if latest_stamp_s is None else max(latest_stamp_s, t_received)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"

        resolved = resolve(candidates, self._cfg)
        winner_label, winner_source = "none", "default"
        raw_v, raw_w = 0.0, 0.0
        if resolved is not None:
            raw_v, raw_w, winner_source, winner_label, _ = resolved

        clamp_reason = ""
        if self._use_safety:
            raw = Twist()
            raw.linear.x = raw_v
            raw.angular.z = raw_w
            clamped, clamp_reason = self._safety.clamp(
                raw,
                tof_m=self._tof_min(),
                tof_rear_m=self._tof_rear_min(),
                lidar_front_min_m=self._lidar_front_min,
                lidar_rear_min_m=self._lidar_rear_min,
                cmd_stamp_s=latest_stamp_s,
                armed=self._armed(now),
                now_s=now,
            )
            out.twist.linear.x = float(clamped.linear.x)
            out.twist.angular.z = float(clamped.angular.z)
        else:
            # Legacy path — plain v_max / w_max clamp only.
            out.twist.linear.x = max(-self._v_max, min(self._v_max, raw_v))
            out.twist.angular.z = max(-self._w_max, min(self._w_max, raw_w))
            if out.twist.linear.x != raw_v or out.twist.angular.z != raw_w:
                clamp_reason = "v_max" if out.twist.linear.x != raw_v else "w_max"

        self.pub_cmd.publish(out)

        # Log transitions only.
        signature = f"{winner_source}:{winner_label}:{clamp_reason}"
        if signature != self._last_winner:
            self._last_winner = signature
            ev = TrialEvent()
            ev.stamp = self.get_clock().now().to_msg()
            ev.event = "arbiter_switch"
            detail = (
                f"winner={winner_source}/{winner_label} "
                f"v={out.twist.linear.x:.3f} w={out.twist.angular.z:.3f}"
            )
            if clamp_reason:
                detail += f" clamp={clamp_reason}"
            ev.detail = detail
            self.pub_ev.publish(ev)
            self.get_logger().info(detail)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Arbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
