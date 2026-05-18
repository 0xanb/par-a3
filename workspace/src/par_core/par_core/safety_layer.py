"""Hard safety clamp — all seven software kill paths in one layer.

Defence in depth. Each check is independent, each can zero the velocity on its
own. None relies on another being present. Designed so that deleting any one
still leaves the robot safe.

Kill paths
----------
H1 ToF hardware Directional. Front ToF (tof_m) < tof_min_m
 zeros FORWARD only — reverse and angular spin
 pass so the recovery FSM can back the chassis
 out of contact. Rear ToF (tof_rear_m) zeros
 REVERSE only. Without directional H1 a single
 front-corner ToF would lock the robot in place
 (the wedge: arbiter clamps v AND w → no
 reverse, no spin, no escape).
H2 LIDAR halo Any forward-cone scan < lidar_stop_m -> zero linear.
 Linear soft-scaling between lidar_slow_m. lidar_stop_m.
H2r LIDAR rear halo When the requested ``cmd.linear.x < 0`` (recovery
 reverse, manual back-up), the same halo is applied
 to the rear ±cone. Without this, the recovery
 FSM's reverse phase has no proximity guard at all
 and can collide with a wall behind the chassis.
H3 Watchdog No call to clamp within watchdog_s -> zero
 velocity on the next call (prevents a stuck thread
 from coasting the robot).
H4 Stale command The intent's timestamp is older than stale_cmd_s ->
 treat as zero.
H5 Accel limiter |dv/dt| and |dw/dt| clamped to a configurable
 maximum per call (also prevents wheel-slip lurches).
H6 Speed cap |v| and |w| clamped to v_max, w_max.
H7 Deadman Caller must pass armed=True; if False, zero.

Usage
-----
Instantiate once per node that writes to cmd_vel. Call clamp on every
command. The layer is stateless across reboots and cheap to run at 50 Hz.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass
class SafetyConfig:
 # Speed caps (H6)
 v_max: float = 0.40 # m/s
 w_max: float = 1.50 # rad/s

 # ToF hardware kill (H1)
 tof_min_m: float = 0.15

 # LIDAR halo (H2) — forward cone
 lidar_slow_m: float = 0.50
 lidar_stop_m: float = 0.25
 lidar_front_cone_deg: float = 30.0 # ± around forward

 # LIDAR halo (H2r) — rear cone, separate from forward.
 # Rear motion is always slow (recovery reverse v=-0.05 m/s, tilt
 # recovery same) so the chassis can creep closer to a rear obstacle
 # than to a forward obstacle. Without separate thresholds the rear
 # halo at the same 0.25 m / 0.50 m stops tilt-recovery reverse in
 # sandwich geometry (front obstacle + rear wall within 0.25 m).
 # introduced after a chassis-tilt incident during early hardware testing
 # showed clamp=lidar_rear_stop blocking TILT_REVERSE.
 lidar_rear_stop_m: float = 0.10
 lidar_rear_slow_m: float = 0.25

 # Watchdog (H3)
 watchdog_s: float = 0.25

 # Stale command (H4)
 stale_cmd_s: float = 0.50

 # Accel limits (H5)
 lin_accel_max: float = 0.5 # m/s^2
 ang_accel_max: float = 3.0 # rad/s^2


class SafetyLayer:
 """All seven software safety paths. Use once per control loop."""

 def __init__(self, config: SafetyConfig | None = None) -> None:
 self.cfg = config or SafetyConfig
 self._last_tick: float | None = None
 self._last_v: float = 0.0
 self._last_w: float = 0.0

 # Public -------------------------------------------------------------
 def clamp(
 self,
 cmd: Twist,
 *,
 tof_m: float | None,
 lidar_front_min_m: float | None,
 cmd_stamp_s: float | None,
 armed: bool,
 now_s: float | None = None,
 lidar_rear_min_m: float | None = None,
 tof_rear_m: float | None = None,
 ) -> tuple[Twist, str]:
 """Clamp ``cmd`` against all kill paths.

 Returns ``(safe_twist, reason)`` where reason is "" when the command
 passed all checks, or the name of the first check that fired.
 """
 now = now_s if now_s is not None else time.monotonic
 prev = self._last_tick
 dt = max(1e-3, now - prev) if prev is not None else 1e-3
 self._last_tick = now

 out = Twist

 # H7 Deadman ------------------------------------------------------
 if not armed:
 return self._zero_and_store(out, "deadman")

 # H3 Watchdog -----------------------------------------------------
 if prev is not None and (now - prev) > self.cfg.watchdog_s:
 return self._zero_and_store(out, "watchdog")

 # H4 Stale command ------------------------------------------------
 if cmd_stamp_s is not None and (now - cmd_stamp_s) > self.cfg.stale_cmd_s:
 return self._zero_and_store(out, "stale")

 # Start from the raw request ------------------------------------
 v = float(cmd.linear.x)
 w = float(cmd.angular.z)
 halo_reason = ""

 # H1 ToF hardware — directional ---------------------------------
 # Front ToF gates FORWARD only; rear ToF gates REVERSE only.
 # Angular spin is never blocked by H1 — the chassis can pivot in
 # place even with a corner ToF triggering, and pivoting is the
 # primary way out of a touch-range pin.
 tof_front_block = (
 tof_m is not None and not math.isnan(tof_m) and tof_m < self.cfg.tof_min_m
 )
 tof_rear_block = (
 tof_rear_m is not None
 and not math.isnan(tof_rear_m)
 and tof_rear_m < self.cfg.tof_min_m
 )
 if tof_front_block and v > 0.0:
 v = 0.0
 halo_reason = "tof"
 if tof_rear_block and v < 0.0:
 v = 0.0
 if not halo_reason:
 halo_reason = "tof_rear"

 # H2 LIDAR halo ---------------------------------------------------
 if lidar_front_min_m is not None and not math.isnan(lidar_front_min_m):
 if lidar_front_min_m < self.cfg.lidar_stop_m:
 v = min(v, 0.0)
 halo_reason = "lidar_stop"
 elif lidar_front_min_m < self.cfg.lidar_slow_m:
 scale = (lidar_front_min_m - self.cfg.lidar_stop_m) / (
 self.cfg.lidar_slow_m - self.cfg.lidar_stop_m
 )
 if v > 0:
 v *= max(0.0, min(1.0, scale))
 halo_reason = "lidar_slow"

 # H2r LIDAR rear halo --------------------------------------------
 # Only consulted when the request is reverse (v < 0). The recovery
 # FSM's reverse phase and the anomaly TILT_REVERSE are the primary
 # consumers; without this, the H2 forward halo would silently
 # allow back-into-wall collisions.
 #
 # Uses ``lidar_rear_stop_m`` / ``lidar_rear_slow_m`` distinct from
 # the forward halo. Rear motion is always slow (recovery v=-0.10,
 # tilt v=-0.05) so the chassis can creep closer to a rear obstacle
 # than to a forward one. Without the split, sandwich geometry
 # (front obstacle + rear wall both within 0.25 m) deadlocked the
 # tilt FSM during early hardware testing.
 if (lidar_rear_min_m is not None and not math.isnan(lidar_rear_min_m)
 and v < 0.0):
 if lidar_rear_min_m < self.cfg.lidar_rear_stop_m:
 v = max(v, 0.0)
 halo_reason = "lidar_rear_stop"
 elif lidar_rear_min_m < self.cfg.lidar_rear_slow_m:
 scale = (lidar_rear_min_m - self.cfg.lidar_rear_stop_m) / (
 self.cfg.lidar_rear_slow_m - self.cfg.lidar_rear_stop_m
 )
 v *= max(0.0, min(1.0, scale))
 halo_reason = "lidar_rear_slow"

 # H6 Speed cap ----------------------------------------------------
 v = max(-self.cfg.v_max, min(self.cfg.v_max, v))
 w = max(-self.cfg.w_max, min(self.cfg.w_max, w))

 # H5 Accel limiter ------------------------------------------------
 dv_max = self.cfg.lin_accel_max * dt
 dw_max = self.cfg.ang_accel_max * dt
 v = self._last_v + max(-dv_max, min(dv_max, v - self._last_v))
 w = self._last_w + max(-dw_max, min(dw_max, w - self._last_w))

 out.linear.x = v
 out.angular.z = w
 self._last_v = v
 self._last_w = w
 return out, halo_reason

 # Internal ----------------------------------------------------------
 def _zero_and_store(self, out: Twist, reason: str) -> tuple[Twist, str]:
 out.linear.x = 0.0
 out.angular.z = 0.0
 self._last_v = 0.0
 self._last_w = 0.0
 return out, reason
