"""par_qr_nav.command_interpreter — DetectionEvent (source='qr') -> CommandIntent.

Subscribes: /par/detections par_msgs/DetectionEvent
Publishes: /par/intents par_msgs/CommandIntent (at ``rate_hz`` while armed)

Behaviour:
 * Receives DetectionEvent(source="qr", payload=<verb>) from qr_detector.
 * Maps the verb to an Intent (see interpreter_core.verb_to_intent).
 * Latches the current intent and republishes it at ``rate_hz`` until a
 different verb arrives.
 * If the latched intent has ``duration_s`` set (TURN_LEFT, TURN_RIGHT,
 U_TURN), the interpreter runs that maneuver for the stated time then
 auto-transitions to the cruise intent (GO). This stops the robot from
 circling forever when the user briefly showed a turn card.
 * Until the first valid QR arrives, no intent is published (arbiter will
 fall through to its default of zero velocity — safe).

Mode-driven runtime note : the earlier MODE_A/B/C/D mode-switch
verbs and confirmation-dance machinery were removed when we reverted from
always-on mode-gating to per-scene SSH launches. The archived design lives
under workspace/src/-archived/mode-driven-runtime/. See
 -revised.
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

from par_msgs.msg import ActiveMode, CommandIntent, DetectionEvent

from.interpreter_core import (
 Intent,
 any_source_moving,
 cruise_intent,
 is_modifier,
 recovering_intent,
 stationary_intent,
 update_motion_record,
 verb_to_intent,
)


# Latched (transient_local) QoS for /par/active_mode — matches every other
# subscriber in the stack (nd_planner, gesture_interpreter, signal_fsm, ).
# Lets a late-joining interpreter see the most recent supervisor publish.
_LATCHED_QOS = QoSProfile(
 depth=1,
 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
 reliability=QoSReliabilityPolicy.RELIABLE,
)


# Mode A vocabulary restriction for the 2-mode pivot. Operator may override
# via the `mode_a_allowed_verbs` ROS param. STOP and GO only — turns.# speed adjusters are rejected to keep the demo flow predictable.
_DEFAULT_MODE_A_VERBS: frozenset[str] = frozenset({"STOP", "GO"})


# Stop-and-turn maneuver phases. Each turn unrolls as:
# IDLE
# -> SETTLE_BEFORE (publish zero, ~_SETTLE_S — let any prior linear motion
# decay to zero before the rotation begins)
# -> ROTATING (publish the rotation intent for its trimmed duration_s)
# -> SETTLE_AFTER (publish zero, ~_SETTLE_S — let the angular decel ramp
# finish so the next intent starts from a stopped state)
# -> IDLE (resume target applied)
#
# Mapping onto the PDF §A.1.4 four-state vocabulary,md
# :
# PDF state | Our internal phase / condition
# ------------- | -----------------------------------------------------
# DRIVING | _PHASE_IDLE + last latched verb was GO/SPEED_*/cross-channel motion
# TURNING | _PHASE_SETTLE_BEFORE | _PHASE_ROTATING | _PHASE_SETTLE_AFTER
# STOPPED | _PHASE_IDLE + last latched verb was STOP
# RECOVERING | one-shot event emitted by recovering_intent on unknown verb
_PHASE_IDLE = "idle"
_PHASE_SETTLE_BEFORE = "settle_before"
_PHASE_ROTATING = "rotating"
_PHASE_SETTLE_AFTER = "settle_after"

# 0.4 s is enough to cover the H5 accel limiter ramping linear or angular
# velocity from peak to zero at the default ~3.5 rad/s² decel and 0.20 m/s
# top speed. Tune via parameter if a tier with much lower decel is added.
_SETTLE_S: float = 0.4


def _zero_intent_for(source: str = "qr") -> Intent:
 return Intent(0.0, 0.0, "STOP", source=source)


class CommandInterpreter(Node):
 def __init__(self) -> None:
 super.__init__("command_interpreter")

 self.declare_parameter("input_topic", "/par/detections")
 self.declare_parameter("output_topic", "/par/intents")
 self.declare_parameter("rate_hz", 10.0)
 self.declare_parameter("settle_s", _SETTLE_S)
 # Vocabulary restriction applied while /par/active_mode == "A". Default
 # is the 2-mode pivot vocabulary (STOP, GO). Set to an empty list to
 # disable the restriction entirely, or override with any subset of the
 # seven supported verbs.
 self.declare_parameter("mode_a_allowed_verbs", ["STOP", "GO"])
 in_topic = self.get_parameter("input_topic").value
 out_topic = self.get_parameter("output_topic").value
 self._rate = float(self.get_parameter("rate_hz").value)
 self._settle_s = float(self.get_parameter("settle_s").value)
 allowed_param = list(self.get_parameter("mode_a_allowed_verbs").value or [])
 self._mode_a_allowed_verbs: frozenset[str] = (
 frozenset(allowed_param) if allowed_param else _DEFAULT_MODE_A_VERBS
 )
 # Latest /par/active_mode value. Defaults to "IDLE" so the interpreter
 # behaves as unrestricted before the supervisor (or scene.sh) latches
 # a real mode — matches the pre-pivot behaviour for safety.
 self._active_mode: str = "IDLE"

 self._current: Intent | None = None
 self._last_confidence: float = 0.0
 self._phase_end: float | None = None # monotonic seconds; None = latched indefinitely
 # Resume target after a maneuver finishes. Starts stationary at boot —
 # the operator must command GO before any modifier (turn, speed adjust)
 # takes effect. See is_modifier in interpreter_core.
 self._resume: Intent = stationary_intent
 # Stop-and-turn state machine for finite maneuvers.
 self._phase: str = _PHASE_IDLE
 self._rotation_intent: Intent | None = None
 # Cross-channel motion tracking: {source -> (monotonic_t, linear_x)}.
 # Updated on every CommandIntent seen on /par/intents (incl. our own).
 # Modifier gate uses any_source_moving so a gesture-driven GO unlocks
 # QR modifiers too, and vice versa.
 self._global_motion: dict[str, tuple[float, float]] = {}

 self.sub = self.create_subscription(DetectionEvent, in_topic, self._on_detect, 10)
 self.pub = self.create_publisher(CommandIntent, out_topic, 10)
 self.create_subscription(CommandIntent, out_topic, self._on_intent, 20)
 # Mode A vocabulary restriction (2-mode pivot): when /par/active_mode
 # latches "A", verbs outside `mode_a_allowed_verbs` are coerced to the
 # one-shot RECOVERING event. Other modes (B, IDLE, ) are unrestricted.
 self.create_subscription(
 ActiveMode, "/par/active_mode", self._on_active_mode, _LATCHED_QOS,
 )
 self._timer = self.create_timer(1.0 / self._rate, self._republish)

 self.get_logger.info(
 f"command_interpreter online: sub={in_topic}, pub={out_topic}, "
 f"rate={self._rate} Hz, mode_a_allowed_verbs={sorted(self._mode_a_allowed_verbs)}"
 )

 def _on_active_mode(self, msg: ActiveMode) -> None:
 new_mode = (msg.mode or "").strip
 if new_mode and new_mode != self._active_mode:
 self.get_logger.info(
 f"active_mode -> {new_mode} (qr restriction "
 f"{'on' if new_mode == 'A' else 'off'})"
 )
 self._active_mode = new_mode
 # Scene A standby. When mode A first activates
 # on a cold boot and no QR card has been shown yet, latch a STOP
 # intent at the operator-stop priority (85) so reactive (p=70)
 # does NOT immediately start driving the chassis the moment the
 # gate flips. Operator must explicitly show a GO card to release
 # this latch — matches the demo-day "robot stays parked until I
 # say go" mental model. The latch is replaced cleanly by any
 # subsequent QR verb (GO, STOP, turn) via _on_detect.
 if (
 new_mode == "A"
 and self._current is None
 and self._phase == _PHASE_IDLE
 ):
 self._current = stationary_intent
 self._resume = stationary_intent
 self.get_logger.info(
 f"scene A standby: latched STOP "
 f"(p={self._current.priority}) until operator shows GO"
 )

 def _on_intent(self, msg: CommandIntent) -> None:
 """Observe all /par/intents traffic for cross-channel motion tracking.

 Self-published settle-zero intents are explicitly NOT recorded so they
 do not overwrite a fresh positive cruise — see update_motion_record.
 """
 update_motion_record(
 self._global_motion,
 msg.source,
 float(msg.cmd.linear.x),
 time.monotonic,
 )

 def _on_detect(self, msg: DetectionEvent) -> None:
 if msg.source != "qr":
 return

 # Mode A restricts the vocabulary to STOP/GO (or whatever
 # `mode_a_allowed_verbs` is set to). Verbs outside the set come back as
 # the RECOVERING one-shot from verb_to_intent so the publish path is
 # identical to an unknown payload.
 restrict_to = (
 self._mode_a_allowed_verbs if self._active_mode == "A" else None
 )
 intent = verb_to_intent(msg.payload, restrict_to=restrict_to)
 if intent is None:
 # Unknown verb -> emit a one-shot RECOVERING event-shaped intent
 # (priority 0) so /par/intents shows the recovery moment without
 # latching state.md.
 self.get_logger.info(
 f"unknown QR payload {msg.payload!r} -> RECOVERING event"
 )
 self._publish_intent(recovering_intent, confidence=0.0)
 return

 # Mode A vocabulary restriction: verb_to_intent returns the
 # RECOVERING intent for any verb outside `mode_a_allowed_verbs`.
 # Treat it like an unknown payload — publish the one-shot event,
 # do not latch any motion or fall through into the modifier gate.
 if restrict_to is not None and msg.payload not in restrict_to:
 self.get_logger.info(
 f"QR {msg.payload!r} rejected in mode A "
 f"(allowed={sorted(restrict_to)}) -> RECOVERING event"
 )
 self._publish_intent(intent, confidence=0.0)
 return

 # Modifier verbs (turns, speed adjustments) only apply while the robot
 # is moving. "Moving" is a cross-channel signal: any source (gesture,
 # voice, traffic GREEN, reactive) with a fresh non-zero-velocity intent
 # counts. The operator does not have to use the same channel that
 # started the motion.
 if is_modifier(msg.payload) and not any_source_moving(self._global_motion, time.monotonic):
 self.get_logger.info(
 f"ignored QR {msg.payload}: no source is moving, show GO first (any channel)"
 )
 return

 self._last_confidence = float(msg.confidence)
 if intent.duration_s is not None:
 # Stop-and-turn: enter SETTLE_BEFORE, publish zero so any prior
 # linear motion decays first, then rotate, then settle again
 # before resuming. Robust to whatever w_max / decel the arbiter
 # is using because we let the actual robot stop in between.
 now = time.monotonic
 self._phase = _PHASE_SETTLE_BEFORE
 self._phase_end = now + self._settle_s
 self._rotation_intent = intent
 self._current = _zero_intent_for(intent.source)
 self.get_logger.info(
 f"QR {intent.label} stop-and-turn: settle "
 f"{self._settle_s:.2f}s -> rotate w={intent.angular_z:.2f} "
 f"for {intent.duration_s:.2f}s -> settle {self._settle_s:.2f}s "
 f"-> resume {self._resume.label}"
 )
 else:
 # Latched intent (STOP, GO, SPEED_UP, SPEED_DOWN). Cancels any
 # in-flight maneuver — a new verb mid-turn always wins.
 self._phase = _PHASE_IDLE
 self._phase_end = None
 self._rotation_intent = None
 self._current = intent
 self._resume = intent # this becomes the new resume target
 self.get_logger.info(
 f"QR {intent.label} latched: v={intent.linear_x:.2f} w={intent.angular_z:.2f}"
 )

 def _republish(self) -> None:
 now = time.monotonic

 ROTATION_PHASES = (_PHASE_SETTLE_BEFORE, _PHASE_ROTATING, _PHASE_SETTLE_AFTER)

 # Stop-and-turn state machine for rotation maneuvers. Each tick checks
 # whether the current phase has elapsed and advances to the next.
 # SETTLE_BEFORE and SETTLE_AFTER both publish zero so the robot is
 # stationary at the entry and exit of the rotation phase.
 if self._phase in ROTATION_PHASES and self._phase_end is not None and now >= self._phase_end:
 if self._phase == _PHASE_SETTLE_BEFORE:
 # Begin the rotation itself.
 rot = self._rotation_intent
 assert rot is not None and rot.duration_s is not None
 self._phase = _PHASE_ROTATING
 self._phase_end = now + rot.duration_s
 self._current = rot
 self.get_logger.info(
 f" -> rotating w={rot.angular_z:.2f} for {rot.duration_s:.2f}s"
 )
 elif self._phase == _PHASE_ROTATING:
 # Rotation duration finished; settle so the angular decel
 # ramp completes before we hand back to the resume target.
 self._phase = _PHASE_SETTLE_AFTER
 self._phase_end = now + self._settle_s
 self._current = _zero_intent_for(self._rotation_intent.source if self._rotation_intent else "qr")
 self.get_logger.info(f" -> settling for {self._settle_s:.2f}s")
 else: # _PHASE_SETTLE_AFTER
 # Maneuver fully complete. Choose resume target:
 # 1. Channel latched its own cruise → resume it.
 # 2. Cross-channel motion is happening → cruise(GO).
 # 3. Otherwise stay stopped (whatever _resume was).
 if self._resume.linear_x > 0.0:
 self._current = self._resume
 elif any_source_moving(self._global_motion, now):
 self._current = cruise_intent
 else:
 self._current = self._resume
 self._phase = _PHASE_IDLE
 self._phase_end = None
 self._rotation_intent = None
 self.get_logger.info(
 f"maneuver complete -> {self._current.label}: "
 f"v={self._current.linear_x:.2f} w={self._current.angular_z:.2f}"
 )

 if self._current is None:
 return
 self._publish_intent(self._current, self._last_confidence)

 def _publish_intent(self, intent: Intent, confidence: float) -> None:
 """Publish ``intent`` once on the output topic with the given confidence."""
 out = CommandIntent
 out.stamp = self.get_clock.now.to_msg
 out.source = intent.source
 out.priority = intent.priority
 out.confidence = confidence
 out.cmd.linear.x = intent.linear_x
 out.cmd.angular.z = intent.angular_z
 out.label = intent.label
 self.pub.publish(out)


def main(args=None) -> None:
 rclpy.init(args=args)
 node = CommandInterpreter
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
