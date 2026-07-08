"""par_gesture.gesture_detector — ROS 2 wrapper around MediaPipe Hands.

Subscribes:  /camera/color/image_raw     sensor_msgs/Image   (BEST_EFFORT)
Publishes:   /par/detections             par_msgs/DetectionEvent  (source="gesture")

Vocabulary lives in :mod:`par_gesture.classifier_core` (7 single-hand poses,
sit-down operator at ~0.6–1.0 m). MediaPipe is lazy-imported so ``colcon
test`` does not pull it. See ```` for the
operating posture and for why the dance is the user-visible mode-switch
ack.
"""
from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from par_core.mode_filter import ModeState
from par_core.snapshot import save_snapshot
from par_msgs.msg import ActiveMode, DetectionEvent, TrialEvent

from .classifier_core import HandFrame, classify


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def should_fire(
    candidate: str | None,
    *,
    pending_label: str | None,
    pending_since_s: float | None,
    last_fired_s: float,
    now_s: float,
    hold_seconds: float,
    cooldown_s: float,
) -> bool:
    """Pure-function stability gate (post ). Replaces the legacy
    tick-based ``hold_ticks`` count with wall-clock time so that changes to
    ``rate_hz`` (CPU policy) do NOT silently change the operator-perceived
    hold duration.

    Returns True when:
      1. candidate is not None (classifier produced a verb), AND
      2. candidate equals pending_label (same pose held since pending_since_s), AND
      3. (now_s - pending_since_s) >= hold_seconds (held long enough), AND
      4. (now_s - last_fired_s) >= cooldown_s (cooldown elapsed since last fire).

    The detector resets pending_label / pending_since_s when candidate
    changes; this function sees the result of that reset and returns False
    naturally on the first tick of a new pose.
    """
    if candidate is None or pending_label is None or pending_since_s is None:
        return False
    if candidate != pending_label:
        return False
    if (now_s - pending_since_s) < hold_seconds:
        return False
    if (now_s - last_fired_s) < cooldown_s:
        return False
    return True


class GestureDetector(Node):
    def __init__(self) -> None:
        super().__init__("gesture_detector")

        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/par/detections")
        self.declare_parameter("rate_hz", 10.0)
        # Stability gate: same label must hold for ``hold_seconds`` of
        # wall-clock time before we publish, then a cooldown blocks re-fire.
        # Post : replaced ``hold_ticks`` with ``hold_seconds``
        # so that a change to rate_hz (e.g. CPU-policy reduction from 10
        # to 5 Hz) does not silently halve the hold duration. 0.4 s default
        # matches the legacy hold_ticks=2 at rate_hz=5 (post- production).
        self.declare_parameter("hold_seconds", 0.4)
        self.declare_parameter("cooldown_s", 1.0)
        # MediaPipe Hands tunables. ``max_num_hands=1`` keeps the inference
        # cheap; the classifier expects a single hand frame at a time.
        self.declare_parameter("max_num_hands", 1)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._rate = float(self.get_parameter("rate_hz").value)
        self._hold_seconds = float(self.get_parameter("hold_seconds").value)
        self._cooldown_s = float(self.get_parameter("cooldown_s").value)

        self._bridge = CvBridge()
        self._latest_msg = None
        self._pending_label: str | None = None
        self._pending_since_s: float | None = None
        self._last_fired_s: float = 0.0

        # Mode gate : MediaPipe inference only runs in mode D.
        # 2-mode pivot post-: default_mode="IDLE" means inference does
        # NOT run during mode A — major CPU saving since MediaPipe Hands at
        # 5 Hz dominates the per-core load (history).
        self._mode = ModeState("D", default_mode="IDLE")
        self.create_subscription(
            ActiveMode, "/par/active_mode",
            lambda m: self._mode.update(m.mode),
            _LATCHED_QOS,
        )

        # Lazy-import MediaPipe so test suites do not load it.
        import mediapipe as mp  # noqa: WPS433
        self._mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=int(self.get_parameter("max_num_hands").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            min_tracking_confidence=float(self.get_parameter("min_tracking_confidence").value),
        )

        self.sub = self.create_subscription(
            Image, in_topic, self._on_image,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.pub = self.create_publisher(DetectionEvent, out_topic, 10)
        # /par/events publisher for snapshotter trigger + session_logger record
        # (post + snapshotter integration).
        self.events_pub = self.create_publisher(TrialEvent, "/par/events", 10)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"gesture_detector online (hands): sub={in_topic}, pub={out_topic}, "
            f"rate={self._rate} Hz, hold={self._hold_seconds}s, "
            f"cooldown={self._cooldown_s}s"
        )

    def _on_image(self, msg: Image) -> None:
        # Just stash the latest message reference — DEFER the cv_bridge
        # conversion to _tick. Pre--evening, this callback ran
        # imgmsg_to_cv2 on every incoming frame at OAK sensor rate (~30 Hz).
        # That conversion at 640x480 RGB costs ~70% of one core on the Pi 5
        # and dominates total gesture_detector CPU regardless of rate_hz.
        # The timer at rate_hz=5 only wants 5 conversions per second; doing
        # the work here at 30 Hz wasted ~83% of conversions. With this
        # change, gesture_detector CPU drops from ~70% to ~25-30%.
        self._latest_msg = msg

    def _tick(self) -> None:
        # MediaPipe inference is one of the heaviest ops in the system;
        # skip it entirely when mode D is not active.
        if not self._mode.is_active():
            return
        msg = self._latest_msg
        if msg is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        import cv2  # local to avoid test-time import
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._mp_hands.process(rgb)
        if not result.multi_hand_landmarks:
            # No hand in frame — reset the pending streak so a re-entering
            # operator has to hold the pose freshly.
            self._pending_label = None
            self._pending_since_s = None
            return

        # Pick the largest hand by bounding-box span. Single-hand operation is
        # the documented operating envelope ; two visible hands at
        # sit-down distance is rare and usually means a misclassification.
        primary_landmarks = max(
            result.multi_hand_landmarks,
            key=lambda hl: max(p.x for p in hl.landmark) - min(p.x for p in hl.landmark),
        )

        now_s = self.get_clock().now().nanoseconds * 1e-9
        lm = [(p.x, p.y, p.z) for p in primary_landmarks.landmark]
        # MediaPipe Hands does not expose a per-landmark visibility score the
        # way Pose does; the post-processed presence score is at landmark
        # level inside the C++ graph but not surfaced in the Python wrapper.
        # We treat all landmarks as visible (1.0) and rely on the
        # min_tracking_confidence gate above to reject low-quality frames.
        vis = [1.0] * len(lm)
        hand = HandFrame(t=now_s, landmarks=lm, visibility=vis)
        candidate = classify(hand)

        # Wall-clock streak (post ): track when the candidate first
        # matched, not how many ticks have passed.
        if candidate != self._pending_label:
            self._pending_label = candidate
            self._pending_since_s = now_s

        if not should_fire(
            candidate,
            pending_label=self._pending_label,
            pending_since_s=self._pending_since_s,
            last_fired_s=self._last_fired_s,
            now_s=now_s,
            hold_seconds=self._hold_seconds,
            cooldown_s=self._cooldown_s,
        ):
            return
        self._last_fired_s = now_s
        # Don't reset _pending_since_s — the cooldown gate handles re-fire
        # suppression. Resetting would let the same held pose re-trigger
        # immediately after cooldown without a fresh hold.

        ev = DetectionEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.source = "gesture"
        ev.payload = candidate
        ev.confidence = 0.8
        ev.image_x = float("nan")
        ev.image_y = float("nan")
        ev.distance_m = float("nan")
        ev.bearing_rad = float("nan")
        self.pub.publish(ev)

        # Emit a TrialEvent so the par_eval snapshotter captures the frame and
        # session_logger records the fire. The snapshotter triggers on
        # event="gesture_read" (added to its trigger set in this commit).
        trial_ev = TrialEvent()
        trial_ev.stamp = ev.stamp
        trial_ev.scenario = "gesture"
        trial_ev.event = "gesture_read"
        trial_ev.detail = candidate
        self.events_pub.publish(trial_ev)

        self.get_logger().info(f"gesture -> {candidate}")
        # Save the frame that triggered the fire . The snapshotter
        # provides a parallel capture path; both write to <session_dir>/captures.
        save_snapshot(frame, source="gesture", label=candidate)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GestureDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
