"""par_supervisor.supervisor — ROS wrapper around the simplified FSM.

Owns ``/par/active_mode`` (TRANSIENT_LOCAL latched). Performs cold-boot
self-validate against three required topics, then publishes a single 360
announce CommandIntent at 10 Hz with ``source="supervisor"``,
``priority=60``, ``label="ANNOUNCE_360"`` so the arbiter (priority resolution)
fuses it like any other behaviour intent.

Post pivot: the legacy /buttons subscriber, /leds publisher, and
mode-B CPU policy were removed. Neither /buttons nor /leds is exposed by
the rosbot_ros snap on this robot (the firmware-source research that
defined those topics applies only to husarion/rosbot-firmware/jazzy, which
is not what is running here). Mode switching to A or B is now driven by
``scripts/scene.sh idle | a | d`` which directly publishes /par/active_mode.
The supervisor's only job is BOOT -> SELF_VALIDATE -> READY_ANNOUNCE -> IDLE,
then idle forever. for the full context.

Topic-alive detection uses ``count_publishers`` polled every tick (matches
the arbiter's pattern), so we no longer hold subscriptions just to flag
freshness. Camera topic is /oak/rgb/image_raw on this robot's depthai-snap
stack (was /camera/color/image_raw in the version).

The pure-function FSM lives in ``supervisor_core``. This wrapper is the
boundary between ROS and the core: param resolution, topic-health snapshot,
and side-effect dispatch.

H7 deadman is NOT consulted by the supervisor; the arbiter owns that.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from par_msgs.msg import ActiveMode, CommandIntent

from .supervisor_core import (
    SupervisorConfig,
    SupervisorFSM,
    SupervisorState,
    TopicHealth,
)


# Latched QoS for /par/active_mode — matches the subscribe-side QoS in
# nd_planner, gesture_interpreter, command_interpreter, signal_fsm, etc.
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class SupervisorNode(Node):
    """ROS wrapper around SupervisorFSM."""

    def __init__(self) -> None:
        super().__init__("supervisor")

        self.declare_parameter("validate_timeout_s", 60.0)
        self.declare_parameter("spin_rate_rad_s", 0.3)
        # Open-loop overrun factor ( fix). The 360° readiness
        # spin commands a constant w for 2π/spin_rate_rad_s seconds, but the
        # actual chassis rotation may diverge from the commanded angle
        # because accel/decel ramps, motor friction, and surface drag are
        # not modelled. Tuning: command the spin, measure with a protractor,
        # set factor = current_factor × 360 / observed_deg. On a charged
        # battery / hardwood / 0.3 rad/s: factor 1.07 yielded ~365° (5° over),
        # factor 1.055 trims it to a true 360°.
        self.declare_parameter("spin_overrun_factor", 1.20)
        # was False here for the trial campaign so the
        # chassis would not auto-yaw between trial runs. Default flipped to
        # True ( fix) for production / demo use: the cold-
        # boot 360° rotation gives the operator a visible "ready" signal
        # and lets perception_fusion + vfh_planner converge their first
        # histograms before the user is expected to interact. Trial harness
        # overrides via launch arg announce_enabled:=false (wired through
        # PAR_ANNOUNCE_ENABLED env var in par_a3_runtime.sh).
        self.declare_parameter("announce_enabled", True)

        validate_timeout_s = float(self.get_parameter("validate_timeout_s").value)
        spin_rate_rad_s = float(self.get_parameter("spin_rate_rad_s").value)
        announce_enabled = bool(self.get_parameter("announce_enabled").value)
        spin_overrun_factor = float(self.get_parameter("spin_overrun_factor").value)

        # 2π / spin_rate_rad_s = single-revolution duration; overrun factor
        # compensates for open-loop accel/decel ramps + motor losses so the
        # actual chassis rotation approaches 360°.
        spin_duration_s = (
            (2.0 * 3.141592653589793 * spin_overrun_factor)
            / max(spin_rate_rad_s, 1e-3)
        )
        if not announce_enabled:
            spin_duration_s = 0.0

        self._cfg = SupervisorConfig(
            validate_timeout_s=validate_timeout_s,
            announce_spin_w_rad_s=spin_rate_rad_s,
            announce_spin_duration_s=spin_duration_s,
        )
        self._fsm = SupervisorFSM(self._cfg)

        # Suppress duplicate validate-fail WARN spam.
        self._validate_fail_warned: bool = False

        # Boot timestamp — fed to FSM.tick() so the validate-timeout window
        # is anchored to node startup, not arbitrary "now".
        self._boot_t = time.monotonic()

        # ---- Publishers --------------------------------------------------
        self._mode_pub = self.create_publisher(
            ActiveMode, "/par/active_mode", _LATCHED_QOS,
        )
        self._intent_pub = self.create_publisher(
            CommandIntent, "/par/intents", 10,
        )

        # ---- Timers ------------------------------------------------------
        # 10 Hz core tick: snapshots topic-alive state, invokes FSM,
        # dispatches side effects.
        self._tick_period_s = 0.1
        self.create_timer(self._tick_period_s, self._tick)

        self.get_logger().info(
            f"supervisor online: validate_timeout={validate_timeout_s:.1f}s, "
            f"spin_rate={spin_rate_rad_s:.2f} rad/s "
            f"(duration {spin_duration_s:.2f}s, announce_enabled={announce_enabled})"
        )

    # ---- core tick -------------------------------------------------------

    def _topic_health_snapshot(self) -> list[TopicHealth]:
        """Build the per-tick TopicHealth list using count_publishers().

        count_publishers is a cheap O(1) graph query that mirrors what the
        arbiter does for its own freshness gating. No subscription overhead
        is needed just to flag whether a topic has any publisher.
        """
        snapshot: list[TopicHealth] = []
        for topic in self._cfg.required_topics_alive:
            count = self.count_publishers(topic)
            snapshot.append(
                TopicHealth(
                    name=topic,
                    alive=count >= 1,
                    publisher_count=count,
                )
            )
        return snapshot

    def _tick(self) -> None:
        now = time.monotonic()

        decision = self._fsm.tick(
            now=now,
            topic_healths=self._topic_health_snapshot(),
            boot_t=self._boot_t,
        )

        # On entry into VALIDATE_FAIL, log every missing topic once.
        if (
            decision.new_state == SupervisorState.VALIDATE_FAIL
            and not self._validate_fail_warned
        ):
            self._validate_fail_warned = True
            for health in self._topic_health_snapshot():
                if not health.alive:
                    self.get_logger().warn(
                        f"validate FAIL: {health.name} never published "
                        f"(timeout={self._cfg.validate_timeout_s:.1f}s)"
                    )

        # Mode publish on transition into IDLE only.
        if decision.publish_active_mode is not None:
            self._publish_active_mode(decision.publish_active_mode)

        # Spin intent during READY_ANNOUNCE.
        if decision.publish_intent is not None:
            linear_x, angular_z = decision.publish_intent
            self._publish_announce_intent(linear_x, angular_z)

    # ---- side effects ----------------------------------------------------

    def _publish_active_mode(self, mode: str) -> None:
        msg = ActiveMode()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        msg.reason = "boot"
        self._mode_pub.publish(msg)
        self.get_logger().info(f"/par/active_mode -> {mode} (latched)")

    def _publish_announce_intent(self, linear_x: float, angular_z: float) -> None:
        msg = CommandIntent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = "supervisor"
        msg.priority = 60
        msg.confidence = 1.0
        msg.label = "ANNOUNCE_360"
        msg.cmd.linear.x = float(linear_x)
        msg.cmd.angular.z = float(angular_z)
        self._intent_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
