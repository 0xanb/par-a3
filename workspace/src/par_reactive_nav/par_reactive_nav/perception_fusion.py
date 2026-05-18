"""par_reactive_nav.perception_fusion — LIDAR polar hist + depth channel.

Publishes the fused histogram on a private topic for downstream planners.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
 QoSDurabilityPolicy,
 QoSPresetProfiles,
 QoSProfile,
 QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray

from par_core.mode_filter import ModeState
from par_msgs.msg import ActiveMode

from.vfh_core import VFHConfig, fuse_depth_channel, scan_to_histogram


_LATCHED_QOS = QoSProfile(
 depth=1,
 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
 reliability=QoSReliabilityPolicy.RELIABLE,
)


class PerceptionFusion(Node):
 def __init__(self) -> None:
 super.__init__("perception_fusion")
 self.declare_parameter("scan_topic", "/scan")
 # See signal_perception for the topic-name rationale: Husarion publishes
 # depth on /oak/stereo/image_raw, not /camera/depth/image_raw.
 self.declare_parameter("depth_topic", "/oak/stereo/image_raw")
 self.declare_parameter("hist_topic", "/par/polar_hist")
 self.declare_parameter("n_bins", 72)
 # Default False: depth fusion costs ~25% CPU on the depthai snap
 # (RGBD vs RGB pipeline) and most demo arenas are LIDAR-friendly
 # (chairs, walls — all things in the LIDAR's horizontal scan plane).
 # project_c.launch.py overrides this to True for the dedicated
 # reactive-nav scene where depth fusion catches chair seats / low
 # boxes that LIDAR misses.
 self.declare_parameter("use_depth", False)
 # : allow ablating LIDAR off the polar histogram so the
 # report has a {LIDAR-only, depth-only, both} 2×2 sensor matrix. With
 # use_lidar=False the scan callback substitutes a max-range histogram
 # so depth fusion is the only obstacle source (depth FOV is ±35° front
 # cone; rear bins remain "free" — expected failure mode is a wedge
 # after the first front obstacle since the planner has no rear info).
 self.declare_parameter("use_lidar", True)
 self.declare_parameter("depth_hfov_deg", 69.0)
 # Husarion ROSbot 3 PRO mounts the LIDAR with a 180° yaw relative to
 # base_link (`tf2_echo` confirms RPY [0, 0, π]). The /scan ranges
 # use the LIDAR frame's a=0 as their reference, which without a
 # transform points BEHIND the chassis. Without this offset, bin 0
 # of the polar histogram is the chassis-rear cone and the planner
 # commands "forward" headings that drive the robot into rear walls.
 # Per-deployment overridable via ROS param; 0 for chassis-aligned
 # LIDAR mounts.
 self.declare_parameter("lidar_yaw_offset_rad", math.pi)
 # Self-occlusion guard: the RPLIDAR S2 publishes returns below its
 # documented range_min (0.150 m) from the robot's own structure —
 # 0.078 m hits at -52° and 0.155 m hits at +155-160° (the
 # rear-left chassis edge, observed). These self-
 # returns combined with chassis_half_width_m=0.165 m angular
 # inflation collapse the entire histogram into permanent DEAD_END
 # because the inflation rule says "obstacle inside chassis envelope
 # blocks every direction." Default 0.20 m ensures min_range_m is
 # always strictly greater than chassis_half_width_m so no surviving
 # beam can ever trigger the catastrophic inflation. The H1 ToF +
 # H2 LIDAR halo (lidar_stop_m=0.15) catches anything genuinely
 # closer than 0.20 m at the safety stack rather than the planner.
 self.declare_parameter("min_range_m", 0.20)
 # Depth freshness guard: when use_depth is enabled but the depth
 # publisher (OAK-D snap) freezes, _on_depth stops updating
 # self._depth but the cached array survives indefinitely. Without
 # this watchdog the histogram fuses scan with multi-second-stale
 # depth data, which is the failure mode propagated downstream.
 # 0.3 s allows for one missed depth frame at 5 Hz worst case.
 self.declare_parameter("depth_max_age_s", 0.3)

 self._cfg = VFHConfig(
 n_bins=int(self.get_parameter("n_bins").value),
 min_range_m=float(self.get_parameter("min_range_m").value),
 )
 self._use_depth = bool(self.get_parameter("use_depth").value)
 self._use_lidar = bool(self.get_parameter("use_lidar").value)
 self._depth_hfov = math.radians(float(self.get_parameter("depth_hfov_deg").value))
 self._lidar_yaw_offset = float(self.get_parameter("lidar_yaw_offset_rad").value)
 self._depth_max_age_s = float(self.get_parameter("depth_max_age_s").value)

 self._bridge = CvBridge
 self._depth: np.ndarray | None = None
 self._depth_at: float = 0.0 # monotonic timestamp of last fresh depth frame

 # 1 Hz instrumentation counters. When perception_fusion appears silent
 # in the field, the operator ssh-es in and tails the runtime log to
 # see whether _on_scan is firing at all and how many frames reached
 # the publisher. Without these counters the silent-histogram failure
 # mode (observed in scene_c0_20260509_204{045,913}.log)
 # was undiagnosable from logs alone.
 self._scan_in: int = 0
 self._scan_pub: int = 0
 self._scan_err: int = 0
 self._depth_in: int = 0
 self._depth_skipped_stale: int = 0
 self._last_log_at: float = time.monotonic

 # 2-mode pivot : perception_fusion runs under "Mode A".
 # default_mode="IDLE" keeps it quiet pre-publish (saves CPU + avoids
 # competing with the supervisor's 360 announce).
 self._mode = ModeState("A", default_mode="IDLE")
 self.create_subscription(
 ActiveMode, "/par/active_mode",
 lambda m: self._mode.update(m.mode),
 _LATCHED_QOS,
 )

 self.create_subscription(
 LaserScan, self.get_parameter("scan_topic").value, self._on_scan,
 qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
 )
 self.create_subscription(
 Image, self.get_parameter("depth_topic").value, self._on_depth,
 qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
 )
 self.pub = self.create_publisher(
 Float32MultiArray, self.get_parameter("hist_topic").value, 10,
 )
 self.get_logger.info("perception_fusion online")

 def _on_depth(self, msg: Image) -> None:
 try:
 img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
 if img.dtype == np.uint16:
 img = img.astype(np.float32) / 1000.0
 self._depth = img
 self._depth_at = time.monotonic
 self._depth_in += 1
 except Exception as e:
 self.get_logger.warn(f"depth: {e}")

 def _depth_to_polar(self, depth: np.ndarray) -> np.ndarray:
 """Project the depth image into polar bins matching our LIDAR grid.

 Vectorised: for each column take the minimum valid pixel and scatter
 it into the polar bin for that column's bearing. The previous
 per-column Python loop ran ~640 iterations at ~10 Hz on a Pi 5
 already at load 9, which left the rclpy executor with no time to
 publish the histogram (silent-histogram failure mode observed in
 scene_c0_20260509_204{045,913}.log).
 """
 n = self._cfg.n_bins
 out = np.full(n, np.inf, dtype=np.float32)
 if depth is None or depth.size == 0:
 return out
 depth = depth.astype(np.float32, copy=False)
 # Column-wise minimum of valid (finite, positive) depth values.
 # np.where masks invalid pixels to +inf so.min preserves the
 # smallest *valid* range per column without a Python-level loop.
 valid = np.isfinite(depth) & (depth > 0)
 masked = np.where(valid, depth, np.inf)
 col_min = masked.min(axis=0) # shape (w,)
 # Drop columns with no valid pixels (col_min == +inf) before scatter.
 h, w = depth.shape[:2]
 cols = np.arange(w, dtype=np.float32)
 # Column x → bearing in [-hfov/2, +hfov/2]; camera forward = angle 0.
 bearings = ((cols - w / 2.0) / (w / 2.0)) * (self._depth_hfov / 2.0)
 # : apply the same yaw offset used by LIDAR so depth
 # and LIDAR agree on which polar bin is chassis-forward. Without
 # this, depth values for forward-facing obstacles were scattering into
 # bins 0N/4 ∪ 3N/4N (camera-forward = polar 0), while LIDAR was
 # mapping chassis-forward to bin N/2 (yaw_offset_rad = π for the
 # 180°-flipped LIDAR). `fuse_depth_channel` then min'd depth into the
 # chassis-rear half of the histogram instead of the front, so the
 # planner never saw low obstacles (slipper, book, paperback) that
 # only depth could detect. Diagnosed in the T-04c diag run.
 polar = np.mod(bearings + self._lidar_yaw_offset, 2.0 * math.pi)
 bin_idx = (polar / (2.0 * math.pi) * n).astype(np.int64) % n
 # np.minimum.at applies the reduction at duplicate indices in-place
 # (multiple columns can map to the same bin at low n_bins).
 finite_cols = np.isfinite(col_min)
 if finite_cols.any:
 np.minimum.at(out, bin_idx[finite_cols], col_min[finite_cols])
 return out

 def _on_scan(self, msg: LaserScan) -> None:
 self._scan_in += 1
 if not self._mode.is_active:
 self._maybe_log
 return
 try:
 if self._use_lidar:
 hist = scan_to_histogram(
 list(msg.ranges), msg.angle_min, msg.angle_increment, self._cfg,
 yaw_offset_rad=self._lidar_yaw_offset,
 )
 else:
 # : LIDAR-disabled ablation — substitute an all-clear
 # histogram so depth (if enabled) is the only obstacle source.
 # /scan still drives the tick rate but contributes no bins.
 hist = np.full(self._cfg.n_bins, float("inf"), dtype=float)
 now = time.monotonic
 depth_age = now - self._depth_at if self._depth_at > 0 else float("inf")
 if (self._use_depth and self._depth is not None
 and depth_age <= self._depth_max_age_s):
 depth_polar = self._depth_to_polar(self._depth)
 hist = fuse_depth_channel(hist, depth_polar)
 elif self._use_depth and self._depth is not None:
 # Depth went stale (camera freeze, snap restart). Skip the
 # depth channel rather than poison the histogram with old
 # data — this is the mitigation. LIDAR-only histogram
 # is degraded but safe.
 self._depth_skipped_stale += 1
 m = Float32MultiArray
 # Replace +inf with a large finite value so it survives message round-tripping.
 finite_hist = np.where(np.isfinite(hist), hist, 1e6).astype(np.float32)
 m.data = finite_hist.tolist
 self.pub.publish(m)
 self._scan_pub += 1
 except Exception as e:
 # Without try/except a NumPy or cv_bridge exception kills the
 # callback silently and the histogram stops publishing — exactly
 # the failure mode that left vfh_planner stuck on "no polar_hist"
 # in the scene_c0 logs.
 self._scan_err += 1
 self.get_logger.warn(
 f"scan_to_histogram/fuse failed: {e!r}", throttle_duration_sec=2.0,
 )
 self._maybe_log

 def _maybe_log(self) -> None:
 """Throttled (1 Hz) instrumentation log so the operator can tell at
 a glance whether _on_scan is alive and what fraction of scans
 actually reached the publisher."""
 now = time.monotonic
 if now - self._last_log_at < 1.0:
 return
 self._last_log_at = now
 depth_age = now - self._depth_at if self._depth_at > 0 else float("inf")
 self.get_logger.info(
 f"scan_in={self._scan_in} pub={self._scan_pub} err={self._scan_err} "
 f"depth_in={self._depth_in} stale_skip={self._depth_skipped_stale} "
 f"depth_age={depth_age:.2f}s use_depth={self._use_depth}"
 )
 # Reset 1-second window counters so each log line reports rate, not
 # cumulative since boot.
 self._scan_in = 0
 self._scan_pub = 0
 self._scan_err = 0
 self._depth_in = 0
 self._depth_skipped_stale = 0


def main(args=None) -> None:
 rclpy.init(args=args)
 node = PerceptionFusion
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
