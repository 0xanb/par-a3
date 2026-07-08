"""par_eval.snapshotter — capture OAK RGB frames on triggered events.

Caches the latest /oak/rgb/image_raw frame in memory and writes a JPEG to
``<session_dir>/captures/<seq>_<event>_<source>.jpg`` whenever:
  * a TrialEvent on /par/events has event in {collision, dyn_entered,
    dead_end_seen, manual_stop, trapped, qr_read}, OR
  * a CommandIntent on /par/intents has label.startswith("DEAD_END")
    (debounced to one snap per 5 s).

These captures are pulled by ``scripts/pull_logs.sh`` along with log.txt and
form the visual evidence layer of the Project C trial campaign. Filename is
sortable by timestamp + sequence so the report's "frame grid" figure is one
``ls`` away.

Filename format::

    <Z>_<event>_<source>.jpg
    where Z = HHMMSS_NNN  (millisecond resolution, lex-sorted)

Pure-function helpers ``should_snapshot`` and ``snapshot_filename`` live at
module level for unit testability without ROS.
"""
from __future__ import annotations

import datetime as _dt
import os
import pathlib
import threading
import time


SNAPSHOT_TRIGGER_EVENTS = frozenset((
    "collision",
    "dyn_entered",
    "dead_end_seen",
    "manual_stop",
    "trapped",
    "qr_read",
    "gesture_read",
    # par_anomaly events : IMU-based detection of low-obstacle
    # collisions, chassis tilt, and motor stalls. Each emission triggers
    # a frame capture so the report's failure-mode appendix has visual
    # evidence aligned with the IMU jerk / odom-divergence signatures.
    "tilt",
    "collision_impact",
    "wheel_stall",
    "wheel_impact",
))


def should_snapshot_event(event_name: str) -> bool:
    """True when a TrialEvent.event value should trigger a frame capture."""
    return event_name in SNAPSHOT_TRIGGER_EVENTS


def should_snapshot_intent(
    label: str,
    *,
    last_snap_at: float | None,
    now_s: float,
    debounce_s: float = 5.0,
) -> bool:
    """True when an intent label should trigger a snapshot (debounced).

    Triggers on any label starting with ``"DEAD_END"`` (catches DEAD_END,
    DEAD_END_LS2, DEAD_END_WEDGE — both VFH+ and ND vocabulary). Debounces
    to one snap per ``debounce_s`` so the spam of recovery-cycle DEAD_END
    intents doesn't fill the disk.
    """
    if not label.startswith("DEAD_END"):
        return False
    if last_snap_at is None:
        return True
    return (now_s - last_snap_at) >= debounce_s


def snapshot_filename(
    event: str,
    source: str = "-",
    seq: int = 0,
    *,
    now: _dt.datetime | None = None,
) -> str:
    """Build a sortable filename for the capture."""
    now = now or _dt.datetime.now()
    stamp = now.strftime("%H%M%S") + f"_{int(now.microsecond / 1000):03d}"
    safe_event = event.replace("/", "_").replace(" ", "_")
    safe_source = (source or "-").replace("/", "_").replace(" ", "_")
    return f"{stamp}_{seq:04d}_{safe_event}_{safe_source}.jpg"


def _captures_dir() -> pathlib.Path:
    """Mirror session_logger's session_dir contract; captures go alongside log.txt."""
    env = os.environ.get("PAR_A3_SESSION_DIR")
    if env:
        path = pathlib.Path(env).expanduser()
    else:
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M")
        path = pathlib.Path("~/par-a3-logs").expanduser() / f"session_{stamp}"
    path = path / "captures"
    path.mkdir(parents=True, exist_ok=True)
    return path


def _build_node():
    """Lazy ROS imports so unit tests can exercise the helpers above."""
    import cv2  # noqa: PLC0415
    import rclpy  # noqa: PLC0415
    from cv_bridge import CvBridge  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415
    from rclpy.qos import QoSPresetProfiles  # noqa: PLC0415
    from sensor_msgs.msg import Image  # noqa: PLC0415

    from par_msgs.msg import CommandIntent, TrialEvent  # noqa: PLC0415

    class Snapshotter(Node):
        def __init__(self) -> None:
            super().__init__("snapshotter")
            self.declare_parameter("image_topic", "/oak/rgb/image_raw")
            self.declare_parameter("intent_debounce_s", 5.0)
            self.declare_parameter("jpeg_quality", 85)

            self._captures = _captures_dir()
            self._bridge = CvBridge()
            self._latest_frame = None  # cv2 image
            self._frame_lock = threading.Lock()
            self._seq = 0
            self._last_intent_snap_at: float | None = None
            self._intent_debounce_s = float(
                self.get_parameter("intent_debounce_s").value
            )
            self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

            self.create_subscription(
                Image,
                self.get_parameter("image_topic").value,
                self._on_image,
                qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
            )
            self.create_subscription(TrialEvent, "/par/events", self._on_event, 50)
            self.create_subscription(CommandIntent, "/par/intents",
                                     self._on_intent, 50)
            self.get_logger().info(f"snapshotter online -> {self._captures}")

        def _on_image(self, msg) -> None:
            try:
                with self._frame_lock:
                    self._latest_frame = self._bridge.imgmsg_to_cv2(
                        msg, desired_encoding="bgr8"
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f"image conversion failed: {exc}", throttle_duration_sec=2.0
                )

        def _on_event(self, msg) -> None:
            if not should_snapshot_event(msg.event):
                return
            source = getattr(msg, "scenario", "") or "event"
            self._capture(msg.event, source)

        def _on_intent(self, msg) -> None:
            now_s = time.monotonic()
            if not should_snapshot_intent(
                msg.label,
                last_snap_at=self._last_intent_snap_at,
                now_s=now_s,
                debounce_s=self._intent_debounce_s,
            ):
                return
            self._last_intent_snap_at = now_s
            self._capture(msg.label, msg.source or "intent")

        def _capture(self, event: str, source: str) -> None:
            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                self.get_logger().warn(
                    f"snapshot trigger '{event}' but no frame cached yet"
                )
                return
            self._seq += 1
            fname = snapshot_filename(event, source=source, seq=self._seq)
            path = str(self._captures / fname)
            try:
                cv2.imwrite(
                    path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
                )
                self.get_logger().info(f"snapshot {fname}")
            except Exception as exc:
                self.get_logger().warn(f"snapshot write failed: {exc}")

    return rclpy, Snapshotter


def main(args=None) -> None:
    rclpy, Snapshotter = _build_node()
    rclpy.init(args=args)
    node = Snapshotter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
