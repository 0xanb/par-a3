"""par_gesture.gesture_interpreter — gesture labels -> CommandIntent.

Behaviour mirrors :mod:`par_qr_nav.command_interpreter`:
* Fire-and-forget DetectionEvents arrive here episodically.
* We latch the last valid intent and republish it at ``rate_hz`` so the
 arbiter keeps seeing fresh data between gesture events.
* Turn intents are in-place, time-bounded maneuvers that auto-transition
 back to cruise (GO) — latching a turn indefinitely would make the robot
 circle forever, which is never what the user meant by "turn left".

EMERGENCY_STOP keeps priority=95 so a raised fist still beats every other
behaviour at the arbiter.
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
from par_msgs.msg import ActiveMode, CommandIntent, DetectionEvent

from.interpreter_core import (
 GestureIntent,
 any_source_moving,
 cruise_intent,
 intent_for,
 is_modifier,
 stationary_intent,
)


_LATCHED_QOS = QoSProfile(
 depth=1,
 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
 reliability=QoSReliabilityPolicy.RELIABLE,
)


class GestureInterpreter(Node):
 def __init__(self) -> None:
 super.__init__("gesture_interpreter")
 self.declare_parameter("detection_topic", "/par/detections")
 self.declare_parameter("intent_topic", "/par/intents")
 self.declare_parameter("rate_hz", 10.0)

 self._rate = float(self.get_parameter("rate_hz").value)
 self._current: GestureIntent | None = None
 self._confidence: float = 0.0
 self._phase_end: float | None = None
 # Resume target — starts stationary. Modifier gestures (turns, speed
 # adjusts) are ignored until any source commands motion (cross-channel).
 self._resume: GestureIntent = stationary_intent
 self._global_motion: dict[str, tuple[float, float]] = {}

 # Mode gate: gesture interpreter only emits in mode D.
 # 2-mode pivot : operator enters via scene.sh d which still
 # publishes /par/active_mode = "D" (legacy code path preserved).
 # default_mode="IDLE" silences this gate during the supervisor's
 # 360 announce window.
 self._mode = ModeState("D", default_mode="IDLE")
 self.create_subscription(
 ActiveMode, "/par/active_mode",
 lambda m: self._mode.update(m.mode),
 _LATCHED_QOS,
 )

 self.create_subscription(
 DetectionEvent, self.get_parameter("detection_topic").value,
 self._on_detect, 10,
 )
 intent_topic = self.get_parameter("intent_topic").value
 self.pub = self.create_publisher(CommandIntent, intent_topic, 10)
 self.create_subscription(CommandIntent, intent_topic, self._on_intent, 20)
 self._timer = self.create_timer(1.0 / self._rate, self._republish)
 self.get_logger.info(f"gesture_interpreter online: rate={self._rate} Hz")

 def _on_intent(self, msg: CommandIntent) -> None:
 self._global_motion[msg.source] = (time.monotonic, float(msg.cmd.linear.x))

 def _on_detect(self, msg: DetectionEvent) -> None:
 if msg.source != "gesture":
 return
 intent = intent_for(msg.payload)
 if intent is None:
 self.get_logger.info(f"ignoring unknown gesture: {msg.payload!r}")
 return

 # Modifier gestures require *any* source to be moving (cross-channel).
 if is_modifier(msg.payload) and not any_source_moving(self._global_motion, time.monotonic):
 self.get_logger.info(
 f"ignored gesture {msg.payload}: no source is moving, show GO first (any channel)"
 )
 return

 self._current = intent
 self._confidence = float(msg.confidence)
 if intent.duration_s is not None:
 self._phase_end = time.monotonic + intent.duration_s
 self.get_logger.info(
 f"gesture {intent.label} maneuver: v={intent.linear_x:.2f} "
 f"w={intent.angular_z:.2f} for {intent.duration_s:.2f}s -> "
 f"then resume {self._resume.label}"
 )
 else:
 self._phase_end = None
 self._resume = intent
 self.get_logger.info(
 f"gesture {intent.label} latched: v={intent.linear_x:.2f} w={intent.angular_z:.2f}"
 )

 def _republish(self) -> None:
 if not self._mode.is_active:
 return
 if self._phase_end is not None and time.monotonic >= self._phase_end:
 # Smart resume: prefer own latched cruise; else cross-channel motion
 # → cruise fallback; else stay stationary.
 if self._resume.linear_x > 0.0:
 self._current = self._resume
 elif any_source_moving(self._global_motion, time.monotonic):
 self._current = cruise_intent
 else:
 self._current = self._resume
 self._phase_end = None
 self.get_logger.info(f"maneuver complete -> {self._current.label}")

 if self._current is None:
 return
 out = CommandIntent
 out.stamp = self.get_clock.now.to_msg
 out.source = "gesture"
 out.priority = self._current.priority
 out.confidence = self._confidence
 out.cmd.linear.x = self._current.linear_x
 out.cmd.angular.z = self._current.angular_z
 out.label = self._current.label
 self.pub.publish(out)


def main(args=None) -> None:
 rclpy.init(args=args)
 node = GestureInterpreter
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
