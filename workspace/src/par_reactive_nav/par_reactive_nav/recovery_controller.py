"""par_reactive_nav.recovery_controller — escape dead ends deterministically.

Listens to the VFH planner's DEAD_END label and, when it persists, overrides
with a scripted reverse + 180° sequence. Uses a small sub-FSM.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from par_core.mode_filter import ModeState
from par_msgs.msg import ActiveMode, CommandIntent, TrialEvent


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


# Pure-function predicates extracted from the post- _on_intent for
# unit-testability without rclpy spin-up. Both VFH+ ("DEAD_END", "FORWARD")
# and ND ("DEAD_END_LS2", "DEAD_END_WEDGE", "HSGR") vocabularies are accepted.

_SPIN_EARLY_EXIT_LABELS = ("FORWARD", "HSGR")
_SPIN_EARLY_EXIT_V_THRESHOLD = 0.10


def is_dead_end_label(label: str) -> bool:
    """True for any planner label that should arm the recovery FSM."""
    return label.startswith("DEAD_END")


def should_early_exit_spin(phase: str, label: str, linear_x: float) -> bool:
    """True when the planner reports a clean forward path and the FSM is
    currently spinning — recovery should release back to the planner."""
    return (
        phase == "spin"
        and label in _SPIN_EARLY_EXIT_LABELS
        and float(linear_x) > _SPIN_EARLY_EXIT_V_THRESHOLD
    )


def should_escalate_trapped(consecutive_recoveries: int,
                             threshold: int = 3) -> bool:
    """True when ``consecutive_recoveries`` reaches the trap threshold.

    Counter is incremented in ``_enter_reverse`` and reset by sustained
    clean-forward observation (5+ s of FORWARD/HSGR with v > 0.10). The
    threshold of 3 means: if the FSM has armed three times without any
    clean-progress interval between them, the geometry is unrecoverable
    and recovery should escalate to TRAPPED rather than loop forever.
    Captures the 4-walls-trap edge case where H1 ToF zeros both axes and
    the FSM cycles indefinitely without producing motion.
    """
    return consecutive_recoveries >= threshold


def is_clean_forward(label: str, linear_x: float) -> bool:
    """A clean-forward observation: planner committed to forward motion at
    substantial v. Mirror of the spin early-exit predicate but without the
    phase requirement — usable from anywhere in the FSM."""
    return label in _SPIN_EARLY_EXIT_LABELS and float(linear_x) > _SPIN_EARLY_EXIT_V_THRESHOLD


class RecoveryController(Node):
    def __init__(self) -> None:
        super().__init__("recovery_controller")
        self.declare_parameter("intent_topic", "/par/intents")
        # : lowered from 0.8 -> 0.3 after live testing showed
        # DEAD_END toggles in/out faster than 0.8 s in tight rooms (planner
        # alternates AVOID/DEAD_END at ~0.1 s tick rate as the histogram
        # noise crosses the dead_end_blocked_frac threshold). At 0.8 s the
        # FSM never armed; at 0.3 s a sustained-2-tick DEAD_END suffices.
        self.declare_parameter("trigger_hold_s", 0.3)
        self.declare_parameter("reverse_duration_s", 1.5)
        # : spin shortened from 2.5 s → 1.0 s and angular velocity
        # reduced from 1.0 → 0.6 rad/s. Old combo rotated ~143°/cycle —
        # multiple cycles read as a full U-turn to the operator. New combo
        # is ~35°/cycle, so the robot reverses → samples a new heading → tries
        # forward again, more like "back up and find a path" behaviour.
        self.declare_parameter("spin_duration_s", 1.0)
        self.declare_parameter("spin_angular_velocity", 0.6)
        # : cooldown after a recovery exit before DEAD_END can
        # re-arm the FSM. Without this, the planner often re-reports
        # DEAD_END within the same scene (the geometry has not yet changed
        # because the robot only moved a few cm), and recovery loops
        # immediately. 1.5 s gives the planner enough time to integrate
        # the new heading into a fresh decision.
        self.declare_parameter("post_recovery_cooldown_s", 1.5)
        # Trap-detection parameters. After ``trap_threshold`` consecutive
        # recovery cycles without a sustained clean-forward interval
        # (``clean_forward_streak_s`` of FORWARD/HSGR with v > 0.10), the FSM
        # escalates to TRAPPED: stops issuing reverse/spin commands, publishes
        # a STOP intent at priority 90 (above the recovery 80, below QR-STOP
        # 95 so the operator can still override), and emits a TrialEvent on
        # /par/events. Operator must lift / reposition the robot and the FSM
        # exits trapped on the next clean-forward observation.
        self.declare_parameter("trap_threshold", 8)
        self.declare_parameter("clean_forward_streak_s", 5.0)

        self._trigger_hold = float(self.get_parameter("trigger_hold_s").value)
        self._reverse_s = float(self.get_parameter("reverse_duration_s").value)
        self._spin_s = float(self.get_parameter("spin_duration_s").value)
        self._spin_w = float(self.get_parameter("spin_angular_velocity").value)
        self._post_recovery_cooldown_s = float(
            self.get_parameter("post_recovery_cooldown_s").value
        )
        self._trap_threshold = int(self.get_parameter("trap_threshold").value)
        self._clean_forward_streak_s = float(
            self.get_parameter("clean_forward_streak_s").value
        )

        self._dead_end_since: float | None = None
        self._recovering_until: float | None = None
        self._phase: str = "idle"            # idle | reverse | spin | trapped
        self._phase_until: float = 0.0
        self._cooldown_until: float = 0.0    # set on recovery exit
        self._consecutive_recoveries: int = 0
        self._clean_forward_since: float | None = None
        self._trapped_emitted: bool = False  # one-shot TrialEvent emission

        # 2-mode pivot post-: reactive recovery is part of "Mode A".
        self._mode = ModeState("A", default_mode="IDLE")
        self.create_subscription(
            ActiveMode, "/par/active_mode",
            lambda m: self._mode.update(m.mode),
            _LATCHED_QOS,
        )

        self.create_subscription(CommandIntent,
                                 self.get_parameter("intent_topic").value,
                                 self._on_intent, 20)
        self.pub = self.create_publisher(CommandIntent,
                                         self.get_parameter("intent_topic").value, 10)
        self.events_pub = self.create_publisher(TrialEvent, "/par/events", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info("recovery_controller online")

    def _on_intent(self, msg: CommandIntent) -> None:
        # Observe the VFH planner's latest verdict to decide when to enter
        # recovery and whether to early-exit a spin (F8 polish: when the
        # planner reports a clear forward valley again, abandon the spin
        # rather than completing the fixed 2.5 s).
        #
        # Operator override : an explicit GO from qr or gesture
        # is a manual intent to resume motion — it must clear TRAPPED state
        # immediately so the operator can drive the robot out of a wedge
        # without lifting it. Without this, once TRAPPED latches, only a
        # 5 s sustained HSGR streak from the planner (which the planner
        # CANNOT publish while TRAPPED holds /cmd_vel=0) can release it.
        if msg.source in ("qr", "gesture") and msg.label == "GO":
            if self._consecutive_recoveries > 0 or self._phase == "trapped":
                self.get_logger().info(
                    f"recovery: operator GO from {msg.source} — clearing trap state"
                )
                self._consecutive_recoveries = 0
                self._trapped_emitted = False
                self._phase = "idle"
                self._dead_end_since = None
                self._clean_forward_since = None
            return
        if msg.source != "reactive":
            return
        if not self._mode.is_active():
            return
        now = time.monotonic()
        # F8 / retighten — early-exit a spin ONLY when the planner
        # reports a clean FORWARD path (label="FORWARD" means |w|<=0.3 in
        # vfh_planner — robot is roughly straight) with substantial forward
        # velocity. AVOID labels (|w|>0.3) used to qualify here, but after
        # the forward-first refactor + cruise_v=0.18 the planner's typical
        # AVOID v sits around 0.07-0.10, comfortably above any reasonable
        # "is this open?" threshold. AVOID just means "I'm steering around
        # something" — that's not a reason to abandon recovery before the
        # spin has actually rotated the robot. Requiring FORWARD AND v>0.10
        # is the legitimate "the path opened up, stop spinning" signal.
        # Track sustained clean-forward observations for trap-counter reset
        # AND for trap-state exit. Only reactive's own intents qualify (recovery
        # publishes its own intents — those don't indicate planner progress).
        if msg.source == "reactive" and is_clean_forward(
            msg.label, float(msg.cmd.linear.x)
        ):
            if self._clean_forward_since is None:
                self._clean_forward_since = now
            elif (now - self._clean_forward_since) >= self._clean_forward_streak_s:
                # Sustained clean forward → reset trap state.
                if self._consecutive_recoveries > 0 or self._phase == "trapped":
                    self.get_logger().info(
                        "recovery: clean-forward streak reached, resetting trap counter"
                    )
                self._consecutive_recoveries = 0
                self._trapped_emitted = False
                if self._phase == "trapped":
                    self._phase = "idle"
                    self._dead_end_since = None
        else:
            self._clean_forward_since = None

        if should_early_exit_spin(self._phase, msg.label, msg.cmd.linear.x):
            self._phase = "idle"
            self._dead_end_since = None
            self._cooldown_until = now + self._post_recovery_cooldown_s
            self.get_logger().info("recovery: spin early-exit (forward open)")
            return
        if self._phase == "trapped":
            # Once trapped, ignore further DEAD_END signals — only sustained
            # clean-forward (operator intervention) exits the trap state.
            return
        if is_dead_end_label(msg.label):
            # Cooldown gate: ignore DEAD_END for a short window after a
            # recovery exit so back-to-back DEAD_END reports from the same
            # geometry do not immediately re-arm the FSM.
            if now < self._cooldown_until:
                self._dead_end_since = None
                return
            if self._dead_end_since is None:
                self._dead_end_since = now
            elif self._phase == "idle" and (now - self._dead_end_since) >= self._trigger_hold:
                self._enter_reverse(now)
        else:
            self._dead_end_since = None

    def _enter_reverse(self, now: float) -> None:
        self._consecutive_recoveries += 1
        if should_escalate_trapped(self._consecutive_recoveries, self._trap_threshold):
            self._phase = "trapped"
            self._dead_end_since = None
            if not self._trapped_emitted:
                ev = TrialEvent()
                ev.stamp = self.get_clock().now().to_msg()
                ev.event = "trapped"
                ev.detail = (
                    f"recovery: {self._consecutive_recoveries} consecutive "
                    f"recoveries without sustained clean-forward; halting"
                )
                self.events_pub.publish(ev)
                self._trapped_emitted = True
                self.get_logger().warn(ev.detail)
            return
        self._phase = "reverse"
        self._phase_until = now + self._reverse_s
        self.get_logger().info(
            f"recovery: reverse (cycle {self._consecutive_recoveries})"
        )

    def _tick(self) -> None:
        if self._phase == "idle":
            return
        if not self._mode.is_active():
            return
        now = time.monotonic()
        out = CommandIntent()
        out.stamp = self.get_clock().now().to_msg()
        # : own source name so the arbiter does not silently
        # overwrite recovery intents with planner intents from the same
        # "reactive" source. arbiter._on_intent stores the latest msg per
        # source key (`_latest_by_source[msg.source] = ...`); when both
        # nodes published as "reactive", planner's 30 Hz fully overwrote
        # recovery's 10 Hz, ignoring the priority gap (planner=70 vs
        # recovery=80). With "recovery" as a separate key the arbiter
        # keeps both intents and picks by priority — recovery wins while
        # active, planner takes over after recovery exits.
        out.source = "recovery"
        out.priority = 80      # above normal reactive
        out.confidence = 1.0

        if self._phase == "trapped":
            # Publish a STOP intent at priority 90 (above recovery 80, below
            # QR-STOP 95) so the arbiter halts the robot until operator
            # intervention. /par/events already carried the trapped event.
            out.priority = 90
            out.cmd.linear.x = 0.0
            out.cmd.angular.z = 0.0
            out.label = "TRAPPED"
            self.pub.publish(out)
            return
        if self._phase == "reverse":
            if now >= self._phase_until:
                self._phase = "spin"
                self._phase_until = now + self._spin_s
                self.get_logger().info("recovery: spin")
            else:
                out.cmd.linear.x = -0.1
                out.cmd.angular.z = 0.0
                out.label = "RECOVER_REVERSE"
                self.pub.publish(out)
                return
        if self._phase == "spin":
            if now >= self._phase_until:
                self._phase = "idle"
                self._dead_end_since = None
                self._cooldown_until = now + self._post_recovery_cooldown_s
                self.get_logger().info("recovery: done")
                return
            out.cmd.linear.x = 0.0
            out.cmd.angular.z = self._spin_w
            out.label = "RECOVER_SPIN"
            self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
