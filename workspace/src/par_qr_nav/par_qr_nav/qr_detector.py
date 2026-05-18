"""par_qr_nav.qr_detector — ROS 2 wrapper around :mod:`.detector_core`.

Subscribes: /camera/color/image_raw sensor_msgs/Image (BEST_EFFORT)
Publishes: /par/detections par_msgs/DetectionEvent

Parameters
----------
input_topic : str default "/camera/color/image_raw"
output_topic : str default "/par/detections"
rate_hz : float default 10.0 — frames per second we actually decode
history_frames : int default 5 — sliding window for temporal voting
min_agree : int default 3 — how many frames must agree before a fire
"""
from __future__ import annotations

import collections
import time
from typing import Deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from par_core.snapshot import save_snapshot
from par_msgs.msg import DetectionEvent

from.detector_core import KNOWN_VERBS, QRResult, detect, temporal_vote


class QRDetector(Node):
 def __init__(self) -> None:
 super.__init__("qr_detector")

 self.declare_parameter("input_topic", "/camera/color/image_raw")
 self.declare_parameter("output_topic", "/par/detections")
 self.declare_parameter("rate_hz", 10.0)
 self.declare_parameter("history_frames", 5)
 self.declare_parameter("min_agree", 3)
 # Same verb can re-fire after this many seconds of no detection. Lets
 # the operator show "TURN_LEFT" twice in a row for a 180° rotation
 # without having to flash a different card in between. Set to 0 to
 # disable the TTL (fire once per unique vote transition, original
 # behaviour).
 self.declare_parameter("dedupe_ttl_s", 2.0)
 # Camera-freeze watchdog: if N consecutive ticks see the same frame
 # (depth-snap or driver wedge — the mode), suppress publishing.
 # Without this guard, a frozen frame containing a GO card keeps the
 # voter firing, the command interpreter keeps republishing, and the
 # robot drives indefinitely on a stale picture. Default 10 ticks is
 # 1.0 s at 10 Hz / 5.0 s at 2 Hz — a generous gate that still beats
 # the arbiter's 0.5 s freshness decay if the camera comes back fast.
 self.declare_parameter("freeze_threshold_ticks", 10)

 in_topic = self.get_parameter("input_topic").value
 out_topic = self.get_parameter("output_topic").value
 self._rate = float(self.get_parameter("rate_hz").value)
 self._history_frames = int(self.get_parameter("history_frames").value)
 self._min_agree = int(self.get_parameter("min_agree").value)
 self._dedupe_ttl_s = float(self.get_parameter("dedupe_ttl_s").value)
 self._freeze_threshold_ticks = int(
 self.get_parameter("freeze_threshold_ticks").value
 )

 self._bridge = CvBridge
 self._det = cv2.QRCodeDetector
 self._latest_bgr: np.ndarray | None = None
 self._history: Deque[list[str]] = collections.deque(maxlen=self._history_frames)
 self._last_fired: str | None = None # one-shot debounce across voter windows
 self._last_fired_s: float = 0.0 # monotonic time of last fire (for TTL)
 # Freeze-detection state. We hash a tiny pixel sample per tick. # count consecutive identical hashes. id is unsafe because cv_bridge
 # may reuse the same numpy buffer across messages.
 self._last_frame_sig: int | None = None
 self._frozen_ticks: int = 0
 self._freeze_logged: bool = False

 self.sub = self.create_subscription(
 Image, in_topic, self._on_image,
 qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
 )
 self.pub = self.create_publisher(DetectionEvent, out_topic, 10)
 self._timer = self.create_timer(1.0 / self._rate, self._tick)

 self.get_logger.info(
 f"qr_detector online: sub={in_topic}, pub={out_topic}, "
 f"rate={self._rate} Hz, window={self._history_frames}, agree={self._min_agree}"
 )

 # Subscribers ------------------------------------------------------
 def _on_image(self, msg: Image) -> None:
 try:
 self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
 except Exception as e: # cv_bridge errors, corrupt frames, etc.
 self.get_logger.warn(f"cv_bridge failed: {e}")
 self._latest_bgr = None

 # Periodic ---------------------------------------------------------
 def _tick(self) -> None:
 frame = self._latest_bgr
 if frame is None:
 return
 # Camera-freeze guard. Hash a sparse sample (16 bytes from 4 corners
 # + centre) — cheap and changes with any real frame movement, while
 # being immune to id reuse by cv_bridge buffer pooling.
 sig = self._frame_signature(frame)
 if sig == self._last_frame_sig:
 self._frozen_ticks += 1
 else:
 self._frozen_ticks = 0
 self._freeze_logged = False
 self._last_frame_sig = sig
 if (self._freeze_threshold_ticks > 0
 and self._frozen_ticks >= self._freeze_threshold_ticks):
 if not self._freeze_logged:
 self.get_logger.warn(
 f"camera frozen ({self._frozen_ticks} ticks of identical "
 f"frames at rate={self._rate:.1f} Hz) — suppressing publish "
 f"until a fresh frame arrives"
 )
 self._freeze_logged = True
 return
 results: list[QRResult] = detect(frame, self._det)

 payloads_this_frame = [r.payload for r in results]
 self._history.append(payloads_this_frame)

 voted = temporal_vote(list(self._history), min_agree=self._min_agree)
 if voted is None:
 return
 now_s = time.monotonic
 # Debounce: same label only re-fires if the TTL has elapsed since the
 # previous fire. Keeps a held-up card from flooding, but lets the user
 # re-show it after dedupe_ttl_s to repeat the verb.
 if (voted == self._last_fired
 and self._dedupe_ttl_s > 0.0
 and now_s - self._last_fired_s < self._dedupe_ttl_s):
 return
 self._last_fired = voted
 self._last_fired_s = now_s

 # Prefer the most recent sighting of the voted payload for localisation.
 loc = next((r for r in reversed(results) if r.payload == voted), None)

 ev = DetectionEvent
 ev.stamp = self.get_clock.now.to_msg
 ev.source = "qr"
 ev.payload = voted
 ev.confidence = 1.0 if voted in KNOWN_VERBS else 0.3
 ev.image_x = float(loc.centroid[0]) if loc else float("nan")
 ev.image_y = float(loc.centroid[1]) if loc else float("nan")
 ev.distance_m = float("nan")
 ev.bearing_rad = float("nan")
 self.pub.publish(ev)
 self.get_logger.info(
 f"QR VOTED -> {voted} (conf={ev.confidence:.2f}, frames={len(self._history)})"
 )
 # Save the frame that triggered this vote. Best-effort — a
 # disk-write failure must not block the publish path.
 save_snapshot(frame, source="qr", label=voted)


 @staticmethod
 def _frame_signature(frame: np.ndarray) -> int:
 """Cheap per-frame hash for the freeze-detection guard.

 Samples five fixed pixels (4 corners + centre) and folds them into
 an int. Sufficient to distinguish a real video stream from a frozen
 buffer while costing ~1 µs/tick (compared to.tobytes hashing on
 a 480x640x3 frame which would be ~0.5 ms)."""
 try:
 h, w = frame.shape[:2]
 samples = (
 int(frame[0, 0, 0]),
 int(frame[0, w - 1, 0]),
 int(frame[h - 1, 0, 0]),
 int(frame[h - 1, w - 1, 0]),
 int(frame[h // 2, w // 2, 0]),
 )
 return hash(samples)
 except Exception:
 return 0


def main(args=None) -> None:
 rclpy.init(args=args)
 node = QRDetector
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
