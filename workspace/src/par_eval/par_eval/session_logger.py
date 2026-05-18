"""par_eval.session_logger — single consolidated text log per session.

Subscribes to the noisy plumbing (``/par/intents``, ``/par/detections``,
``/par/events``, ``/par/active_mode``, and ``/rosout``) and writes a single
human-readable text file at ``<session_dir>/log.txt``. One line per event,
columnar layout, monotonic-time sortable.

Design rationale livesmd.

Sample lines::

 14:32:01.234 intent qr STOP priority=85 v=0.00 w=0.00
 14:32:01.301 detect qr STOP confidence=0.95
 14:32:05.789 mode - A reason=boot
 14:32:11.412 event reactive stale_perception detail=""

Pure-function helpers ``format_intent_line`` etc. live in this module so
host tests can verify formatting without ROS imports.
"""
from __future__ import annotations

import datetime as _dt
import os
import pathlib

# ---------------------------------------------------------------------------
# Pure formatting helpers (testable without ROS)
# ---------------------------------------------------------------------------


def _ts(t_s: float) -> str:
 return _dt.datetime.fromtimestamp(t_s).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def format_intent_line(t_s: float, source: str, label: str,
 priority: int, v: float, w: float) -> str:
 return (f"{_ts(t_s):<23} intent {source:<8} {label:<16} "
 f"priority={priority:<3} v={v:0.2f} w={w:0.2f}")


def format_detect_line(t_s: float, source: str, payload: str,
 confidence: float) -> str:
 return (f"{_ts(t_s):<23} detect {source:<8} {payload:<16} "
 f"confidence={confidence:0.2f}")


def format_event_line(t_s: float, source: str, event: str, detail: str = "") -> str:
 detail_field = f" detail=\"{detail}\"" if detail else ""
 return (f"{_ts(t_s):<23} event {source:<8} {event:<16}{detail_field}")


def format_mode_line(t_s: float, mode: str, reason: str) -> str:
 # Mode rows have no per-source provenance; use "-".
 return f"{_ts(t_s):<23} mode {'-':<8} {mode:<16} reason={reason}"


def format_log_line(t_s: float, level: str, name: str, msg: str) -> str:
 return f"{_ts(t_s):<23} log {level:<8} {name:<16} {msg}"


def format_odom_line(t_s: float, x: float, y: float) -> str:
 """Emitted at ~1 Hz; analyze_trial.py integrates distance for coverage."""
 return f"{_ts(t_s):<23} odom {'-':<8} {'pose':<16} x={x:0.3f} y={y:0.3f}"


# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------


def _session_dir -> pathlib.Path:
 """Mirror par_core.snapshot.session_dir without importing it (avoids the
 ROS-laden par_core/__init__.py during host tests). Same env-var contract.
 """
 env = os.environ.get("PAR_A3_SESSION_DIR")
 if env:
 path = pathlib.Path(env).expanduser
 else:
 stamp = _dt.datetime.now.strftime("%Y%m%d_%H%M")
 path = pathlib.Path("~/par-a3-logs").expanduser / f"session_{stamp}"
 path.mkdir(parents=True, exist_ok=True)
 return path


def session_log_path -> pathlib.Path:
 return _session_dir / "log.txt"


# ---------------------------------------------------------------------------
# ROS node (only loaded when run as a script — keeps host tests ROS-free)
# ---------------------------------------------------------------------------


def _build_node:
 """Lazy ROS imports so this file is importable for unit tests outside
 the dev container."""
 import rclpy # noqa: PLC0415
 from nav_msgs.msg import Odometry # noqa: PLC0415
 from rcl_interfaces.msg import Log # noqa: PLC0415
 from rclpy.node import Node # noqa: PLC0415
 from rclpy.qos import ( # noqa: PLC0415
 QoSDurabilityPolicy,
 QoSPresetProfiles,
 QoSProfile,
 QoSReliabilityPolicy,
 )

 from par_msgs.msg import ( # noqa: PLC0415
 ActiveMode,
 CommandIntent,
 DetectionEvent,
 TrialEvent,
 )

 LATCHED_QOS = QoSProfile(
 depth=1,
 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
 reliability=QoSReliabilityPolicy.RELIABLE,
 )

 LOG_LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

 class SessionLogger(Node):
 def __init__(self) -> None:
 super.__init__("session_logger")
 self._path = session_log_path
 self._fh = open(self._path, "a", buffering=1) # line-buffered
 self.get_logger.info(f"session log -> {self._path}")
 # Plumbing subscriptions.
 self.create_subscription(CommandIntent, "/par/intents", self._on_intent, 50)
 self.create_subscription(DetectionEvent, "/par/detections", self._on_detect, 50)
 self.create_subscription(TrialEvent, "/par/events", self._on_event, 50)
 self.create_subscription(ActiveMode, "/par/active_mode",
 self._on_mode, LATCHED_QOS)
 # /rosout uses a non-standard QoS preset; the default depth=10 is
 # plenty for human-paced log inspection.
 self.create_subscription(Log, "/rosout", self._on_log, 10)
 # Odometry — throttled to 1 Hz to keep log volume bounded while
 # giving analyze_trial.py enough samples for path-length integration.
 self.create_subscription(
 Odometry, "/odometry/filtered", self._on_odom,
 qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
 )
 self._last_odom_at: float = 0.0
 self._odom_min_period_s: float = 1.0

 @staticmethod
 def _now_s -> float:
 return _dt.datetime.now.timestamp

 def _write(self, line: str) -> None:
 try:
 self._fh.write(line + "\n")
 except Exception as exc: # disk full, etc.
 self.get_logger.warn(f"session_logger write failed: {exc}")

 def _on_intent(self, msg) -> None:
 self._write(format_intent_line(
 self._now_s, msg.source, msg.label,
 int(msg.priority), float(msg.cmd.linear.x), float(msg.cmd.angular.z),
 ))

 def _on_detect(self, msg) -> None:
 self._write(format_detect_line(
 self._now_s, msg.source, msg.payload, float(msg.confidence),
 ))

 def _on_event(self, msg) -> None:
 self._write(format_event_line(
 self._now_s, getattr(msg, "scenario", "-") or "-",
 msg.event, msg.detail,
 ))

 def _on_mode(self, msg) -> None:
 self._write(format_mode_line(self._now_s, msg.mode, msg.reason))

 def _on_log(self, msg) -> None:
 level = LOG_LEVEL_MAP.get(int(msg.level), str(int(msg.level)))
 self._write(format_log_line(self._now_s, level, msg.name, msg.msg))

 def _on_odom(self, msg) -> None:
 now = self._now_s
 if now - self._last_odom_at < self._odom_min_period_s:
 return
 self._last_odom_at = now
 self._write(format_odom_line(
 now,
 float(msg.pose.pose.position.x),
 float(msg.pose.pose.position.y),
 ))

 def destroy_node(self) -> bool:
 try:
 self._fh.close
 except Exception:
 pass
 return super.destroy_node

 return rclpy, SessionLogger


def main(args=None) -> None:
 rclpy, SessionLogger = _build_node
 rclpy.init(args=args)
 node = SessionLogger
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
