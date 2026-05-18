"""par_anomaly.anomaly_detector — ROS node, three independent detectors.

Thin wrapper around the pure-function predicates in ``detectors.py``.
Each detector owns its own rolling state in this class; the predicates
are stateless.

Detectors wired:

* **Tilt** — subscribes ``/imu_broadcaster/imu`` orientation, publishes
 ``TILT_REVERSE`` intent at priority 95 (v=-0.05 m/s, w=0) +
 ``TrialEvent(event="tilt")``. Reverse drive instead of pure stop so the
 chassis can back off the obstacle that tilted it — pure halt was an
 absorbing state. Hysteresis: 1.0 s of sustained ``is_level`` before
 release.
* **Collision impact** — subscribes ``/imu_broadcaster/imu``
 linear_acceleration.x, publishes ``TrialEvent(event="collision_impact")``
 on jerk spike. Conditioned on the stall predicate: only emits if the
 wheels stop within 0.5 s of the spike (suppresses rumble strips. thresholds). 1.0 s emit cooldown to prevent spam.
* **Wheel stall** — subscribes ``/cmd_vel`` (post-clamp). ``/odometry/filtered`` (EKF chassis velocity), publishes
 ``TrialEvent(event="wheel_stall")`` when cmd average exceeds threshold
 but odom average does not. Uses the cold-start arm-gate pattern.
 5.0 s emit cooldown.

ROS imports are lazy so the package is importable for host tests outside
the dev container (matches ``par_eval.session_logger``).
"""
from __future__ import annotations


def _build_node:
 import collections # noqa: PLC0415
 import math # noqa: PLC0415
 import time # noqa: PLC0415

 import rclpy # noqa: PLC0415
 from geometry_msgs.msg import TwistStamped # noqa: PLC0415
 from nav_msgs.msg import Odometry # noqa: PLC0415
 from rclpy.node import Node # noqa: PLC0415
 from rclpy.qos import QoSPresetProfiles # noqa: PLC0415
 from sensor_msgs.msg import Imu # noqa: PLC0415

 from par_msgs.msg import CommandIntent, TrialEvent # noqa: PLC0415

 from par_anomaly.detectors import ( # noqa: PLC0415
 is_collision_impact,
 is_level,
 is_stalled,
 is_tilted,
 is_wheel_impact,
 quat_to_roll_pitch,
 )

 SENSOR_QOS = QoSPresetProfiles.SENSOR_DATA.value

 class AnomalyDetector(Node):
 def __init__(self) -> None:
 super.__init__("anomaly_detector")
 # Tunable thresholds. Defaults match the plan +.
 self.declare_parameter("tilt_trip_deg", 8.0)
 self.declare_parameter("tilt_clear_deg", 5.0)
 self.declare_parameter("tilt_release_hold_s", 1.0)
 self.declare_parameter("tilt_intent_priority", 95)
 # Trip debounce (fix,). Require N consecutive
 # IMU samples with tilt > tilt_trip_deg before flipping the FSM
 # into "reverse" phase. 100 Hz IMU × 10 samples = 100 ms minimum
 # sustained tilt. Filters the single-sample placement transient
 # the operator generates when hand-placing the chassis (observed
 # in this session: one 80 ms spike to roll=-9.6° during a
 # placement then immediately back to level — without debounce
 # this tripped the full reverse + reverse_clear + 100° spin
 # recovery, which looks like spurious auto-rotation to a level
 # robot just being put down). Real tilt events (riding up onto
 # a low obstacle) last hundreds of ms, well above the debounce
 # window, so they still trip normally.
 self.declare_parameter("tilt_trip_debounce_n", 10)
 # Calibrated: was 8.0; at slow cruise
 # (0.057 m/s the chassis was running at during the white-box
 # impact) the actual jerk was ~5.7 m/s³ — below the old
 # threshold. Lowered to 4.0 to catch slow-tier impacts. The
 # AND-with-stall conditioning still prevents rumble-strip
 # false positives.
 self.declare_parameter("collision_jerk_threshold", 4.0)
 self.declare_parameter("collision_window_n", 5)
 self.declare_parameter("collision_cooldown_s", 1.0)
 self.declare_parameter("stall_window_n", 20)
 self.declare_parameter("stall_cmd_threshold", 0.05)
 self.declare_parameter("stall_odom_threshold", 0.02)
 self.declare_parameter("stall_cooldown_s", 5.0)
 # Fast-window wheel-impact detector — same shape as is_stalled
 # but 6 samples (~300 ms at 20 Hz). Catches transient hits the
 # slow detector would average out. 1.0 s emit cooldown.
 self.declare_parameter("wheel_impact_window_n", 6)
 self.declare_parameter("wheel_impact_cooldown_s", 1.0)
 # Tilt FSM (evolution; hardened). On tilt:
 # 1. REVERSE — drive backward at tilt_reverse_v while tilted.
 # 2. REVERSE_CLEAR — chassis level, keep reversing for
 # tilt_reverse_clear_s to gain clearance from the obstacle.
 # 3. SPIN — rotate at tilt_spin_w for tilt_spin_duration_s
 # so the next forward attempt picks a different heading.
 # 4. RELEASE — return to idle; downstream nav resumes.
 # Field finding: -0.05 m/s reverse barely overcame
 # rolling resistance and the spin happened while still adjacent
 # to the obstacle, so the robot looped reverse→spin→re-hit. The
 # reverse_clear phase + stronger reverse + larger scan break that
 # loop.
 # Lowered from -0.12 → -0.08 ( PM, ): -0.12 +
 # operator physically holding the robot caused a motor-current
 # transient that browned out the Pi 5. -0.08 still 60% stronger
 # than the original -0.05 baseline but well under the brownout
 # threshold during a stall against a hold.
 self.declare_parameter("tilt_reverse_v", -0.08)
 self.declare_parameter("tilt_reverse_clear_s", 1.0)
 self.declare_parameter("tilt_spin_w", 0.8)
 self.declare_parameter("tilt_spin_duration_s", 2.2)
 # Settle-after-recovery (fix,). After the spin
 # phase the chassis is publishing w=0 v=0 for this many seconds
 # before falling back to idle, so perception_fusion's polar
 # histogram has time to fully refresh and reactive does not
 # re-evaluate against a stale wedge-shaped histogram. Without
 # this, reactive often resumes FORWARD inside a 50 ms window
 # where the histogram still reflects the obstacle in front,
 # finds a hairline valley, drives in, and re-trips the tilt.
 self.declare_parameter("settle_after_recovery_s", 0.5)
 # Wedge-repeating detection (fix,). If TILT_REVERSE
 # phase entry happens N times within a rolling window, the
 # chassis is stuck in a wedge it cannot escape via the
 # reverse+spin recovery. Transition into a "wedged" terminal
 # state: first publish a full 360° announce spin so the
 # operator sees the chassis spinning in place (visible "I am
 # giving up" signal), then publish a hard STOP at the tilt
 # priority indefinitely. Operator must rescue + restart the
 # service to recover.
 self.declare_parameter("wedge_max_trips", 3)
 self.declare_parameter("wedge_window_s", 30.0)
 self.declare_parameter("wedge_announce_w", 0.3)
 # Duration matches supervisor's tuned 360° (2π × 1.055 / 0.3
 # ≈ 22.1 s on hardwood / 0.3 rad/s). Bake the same overrun
 # factor here so the wedge announce and the cold-boot announce
 # rotate the same physical angle.
 self.declare_parameter("wedge_announce_duration_s", 22.1)
 # Stall-driven recovery : when the planner commands
 # forward but odom shows no motion (wheels blocked by something
 # low/invisible to LIDAR — slipper lip, doorframe sill, chair
 # base spider), reuse the same reverse → reverse_clear → spin
 # recovery as tilt. Triggered from the stall predicate that
 # was previously advisory-only.
 self.declare_parameter("stall_react", True)

 self._trip_rad = math.radians(
 float(self.get_parameter("tilt_trip_deg").value),
 )
 self._clear_rad = math.radians(
 float(self.get_parameter("tilt_clear_deg").value),
 )
 self._release_hold_s = float(
 self.get_parameter("tilt_release_hold_s").value,
 )
 self._intent_priority = int(
 self.get_parameter("tilt_intent_priority").value,
 )
 self._trip_debounce_n = int(
 self.get_parameter("tilt_trip_debounce_n").value,
 )
 self._collision_jerk = float(
 self.get_parameter("collision_jerk_threshold").value,
 )
 self._collision_window_n = int(
 self.get_parameter("collision_window_n").value,
 )
 self._collision_cooldown_s = float(
 self.get_parameter("collision_cooldown_s").value,
 )
 self._stall_n = int(self.get_parameter("stall_window_n").value)
 self._stall_cmd_threshold = float(
 self.get_parameter("stall_cmd_threshold").value,
 )
 self._stall_odom_threshold = float(
 self.get_parameter("stall_odom_threshold").value,
 )
 self._stall_cooldown_s = float(
 self.get_parameter("stall_cooldown_s").value,
 )
 self._tilt_reverse_v = float(
 self.get_parameter("tilt_reverse_v").value,
 )
 self._tilt_spin_w = float(self.get_parameter("tilt_spin_w").value)
 self._tilt_spin_duration_s = float(
 self.get_parameter("tilt_spin_duration_s").value,
 )
 self._tilt_reverse_clear_s = float(
 self.get_parameter("tilt_reverse_clear_s").value,
 )
 self._settle_after_recovery_s = float(
 self.get_parameter("settle_after_recovery_s").value,
 )
 self._wedge_max_trips = int(
 self.get_parameter("wedge_max_trips").value,
 )
 self._wedge_window_s = float(
 self.get_parameter("wedge_window_s").value,
 )
 self._wedge_announce_w = float(
 self.get_parameter("wedge_announce_w").value,
 )
 self._wedge_announce_duration_s = float(
 self.get_parameter("wedge_announce_duration_s").value,
 )
 self._stall_react = bool(self.get_parameter("stall_react").value)
 self._wheel_impact_n = int(
 self.get_parameter("wheel_impact_window_n").value,
 )
 self._wheel_impact_cooldown_s = float(
 self.get_parameter("wheel_impact_cooldown_s").value,
 )
 self._last_wheel_impact_emit_at: float = 0.0

 # Tilt FSM state:
 # - idle: nothing active
 # - reverse: chassis tilted; drive backward at tilt_reverse_v
 # - reverse_clear: chassis level; keep reversing for
 # tilt_reverse_clear_s to gain clearance from the obstacle
 # - spin: rotate at tilt_spin_w to change heading before normal
 # nav resumes
 # - settle: publish zero v=0 w=0 for settle_after_recovery_s
 # so perception_fusion's histogram fully refreshes before
 # reactive re-evaluates the path. Prevents the "planner
 # re-enters the same dead-end against a stale histogram"
 # thrash.
 # - wedged_announce: chassis is in a repeating wedge that
 # recovery cannot escape. Publish a 360° announce spin so
 # the operator sees a visible "giving up" signal, then drop
 # into wedged_park.
 # - wedged_park: terminal STOP at the tilt priority until the
 # service is restarted by the operator.
 self._tilt_phase: str = "idle"
 self._level_since: float | None = None
 self._reverse_clear_until_s: float | None = None
 self._spin_until_s: float | None = None
 self._settle_until_s: float | None = None
 self._wedge_announce_until_s: float | None = None
 self._last_roll_rad: float = 0.0
 self._last_pitch_rad: float = 0.0
 # Consecutive-trip streak counter for the trip debounce.
 self._trip_streak: int = 0
 # Rolling history of TILT_REVERSE phase entries (monotonic
 # seconds) for the wedge-repeating gate.
 self._tilt_trip_history: list[float] = []

 # Collision state (accel ring + emit cooldown)
 self._accel_x_ring: collections.deque = collections.deque(maxlen=20)
 self._last_collision_emit_at: float = 0.0
 self._last_jerk_spike_at: float | None = None

 # Stall state (cmd + odom rings, with arm gate)
 self._cmd_v_ring: list[float] = []
 self._odom_v_ring: list[float] = []
 self._stall_armed: bool = False
 self._stall_arm_time: float | None = None
 self._last_cmd_v_seen: float = 0.0
 self._last_odom_v_seen: float = 0.0
 self._last_stall_emit_at: float = 0.0

 # Stats (1 Hz instrumentation)
 self._imu_count: int = 0
 self._cmd_count: int = 0
 self._odom_count: int = 0

 self.create_subscription(
 Imu, "/imu_broadcaster/imu", self._on_imu, SENSOR_QOS,
 )
 self.create_subscription(
 TwistStamped, "/cmd_vel", self._on_cmd_vel, SENSOR_QOS,
 )
 self.create_subscription(
 Odometry, "/odometry/filtered", self._on_odom, SENSOR_QOS,
 )
 self.events_pub = self.create_publisher(TrialEvent, "/par/events", 10)
 self.intent_pub = self.create_publisher(CommandIntent, "/par/intents", 10)

 # 10 Hz: republish TILT_REVERSE while tripped (drives the
 # chassis off the obstacle that tilted it; v=-0.05 default).
 self.create_timer(0.1, self._tick_tilt_intent)
 # 1 Hz: instrumentation.
 self.create_timer(1.0, self._tick_log)

 self.get_logger.info(
 f"anomaly_detector online — tilt trip={self._trip_rad:.3f}rad "
 f"jerk_thr={self._collision_jerk:.1f}m/s3 "
 f"stall_n={self._stall_n}",
 )

 # ---- IMU callback (100 Hz) -----------------------------------------

 def _on_imu(self, msg) -> None:
 self._imu_count += 1
 now = time.monotonic
 q = msg.orientation
 roll, pitch = quat_to_roll_pitch(q.x, q.y, q.z, q.w)
 self._last_roll_rad = roll
 self._last_pitch_rad = pitch

 # ---- Tilt ----
 tripped_now = is_tilted(
 roll_rad=roll,
 pitch_rad=pitch,
 max_roll_rad=self._trip_rad,
 max_pitch_rad=self._trip_rad,
 )
 level_now = is_level(
 roll_rad=roll,
 pitch_rad=pitch,
 clear_roll_rad=self._clear_rad,
 clear_pitch_rad=self._clear_rad,
 )
 # Trip debounce: count consecutive over-threshold samples and only
 # flip the FSM into "reverse" once the streak crosses
 # tilt_trip_debounce_n. A single-sample placement transient (the
 # operator hand-placing the chassis) clears the streak on the
 # next level reading and never trips. A real wedge tilt lasts
 # hundreds of ms and trips normally.
 if tripped_now:
 self._trip_streak += 1
 else:
 self._trip_streak = 0
 tripped_debounced = self._trip_streak >= self._trip_debounce_n

 # FSM transitions:
 # idle → reverse: chassis goes from level to tilted (debounced)
 # reverse → reverse_clear: chassis level for release_hold_s
 # reverse_clear → spin: tilt_reverse_clear_s elapsed since level
 # spin → idle: spin_duration_s elapsed
 # reverse_clear|spin → reverse (re-trip): chassis tilts again
 # wedged_park is terminal — never advance the FSM out of it.
 if self._tilt_phase == "wedged_park":
 pass
 elif tripped_debounced and self._tilt_phase != "reverse":
 if self._tilt_phase == "idle":
 self._emit_event(
 "tilt",
 f"roll={math.degrees(roll):.1f} "
 f"pitch={math.degrees(pitch):.1f}",
 )
 self._tilt_phase = "reverse"
 self._level_since = None
 self._reverse_clear_until_s = None
 self._spin_until_s = None
 self._settle_until_s = None
 # Record the trip time and prune older entries outside the
 # rolling wedge-detection window.
 self._tilt_trip_history.append(now)
 self._tilt_trip_history = [
 t for t in self._tilt_trip_history
 if (now - t) <= self._wedge_window_s
 ]
 self.get_logger.warn(
 f"TILT reverse: roll={math.degrees(roll):.1f}° "
 f"pitch={math.degrees(pitch):.1f}° "
 f"(trip {len(self._tilt_trip_history)}/{self._wedge_max_trips} "
 f"in {self._wedge_window_s:.0f}s)",
 )
 elif self._tilt_phase == "reverse":
 if level_now:
 if self._level_since is None:
 self._level_since = now
 elif (now - self._level_since) >= self._release_hold_s:
 # Level sustained — keep reversing for
 # tilt_reverse_clear_s to gain clearance before spin.
 self._tilt_phase = "reverse_clear"
 self._reverse_clear_until_s = (
 now + self._tilt_reverse_clear_s
 )
 self._level_since = None
 self.get_logger.info(
 f"TILT level for {self._release_hold_s:.1f}s → "
 f"reverse_clear for {self._tilt_reverse_clear_s:.1f}s",
 )
 else:
 self._level_since = None
 elif self._tilt_phase == "reverse_clear":
 if (
 self._reverse_clear_until_s is not None
 and now >= self._reverse_clear_until_s
 ):
 self._tilt_phase = "spin"
 self._spin_until_s = now + self._tilt_spin_duration_s
 self._reverse_clear_until_s = None
 self.get_logger.info(
 f"TILT reverse_clear done → spin "
 f"({self._tilt_spin_duration_s:.1f}s @ "
 f"{self._tilt_spin_w:.2f}rad/s)",
 )
 elif self._tilt_phase == "spin":
 if self._spin_until_s is not None and now >= self._spin_until_s:
 self._spin_until_s = None
 # Wedge gate: if we have tripped wedge_max_trips times
 # in the rolling window, recovery has not been working.
 # Skip the settle/idle path and enter the wedged
 # announce → park sequence so the operator gets a
 # visible signal and the chassis stops trying to drive
 # itself out of an inescapable position.
 if len(self._tilt_trip_history) >= self._wedge_max_trips:
 self._tilt_phase = "wedged_announce"
 self._wedge_announce_until_s = (
 now + self._wedge_announce_duration_s
 )
 self._emit_event(
 "wedge_repeating",
 f"trips={len(self._tilt_trip_history)} "
 f"in {self._wedge_window_s:.0f}s",
 )
 self.get_logger.warn(
 f"WEDGE: {len(self._tilt_trip_history)} tilt trips "
 f"in {self._wedge_window_s:.0f}s — entering "
 f"wedged_announce ({self._wedge_announce_duration_s:.1f}s "
 f"@ {self._wedge_announce_w:.2f}rad/s) → wedged_park",
 )
 else:
 self._tilt_phase = "settle"
 self._settle_until_s = (
 now + self._settle_after_recovery_s
 )
 self.get_logger.info(
 f"TILT spin done → settle "
 f"{self._settle_after_recovery_s:.2f}s",
 )
 elif self._tilt_phase == "settle":
 if (
 self._settle_until_s is not None
 and now >= self._settle_until_s
 ):
 self._tilt_phase = "idle"
 self._settle_until_s = None
 self.get_logger.info("TILT recovery done")
 elif self._tilt_phase == "wedged_announce":
 if (
 self._wedge_announce_until_s is not None
 and now >= self._wedge_announce_until_s
 ):
 self._tilt_phase = "wedged_park"
 self._wedge_announce_until_s = None
 self.get_logger.warn(
 "WEDGE: announce spin done → wedged_park (terminal). "
 "Operator must rescue + restart par-a3-runtime."
 )

 # ---- Collision impact ----
 self._accel_x_ring.append(float(msg.linear_acceleration.x))
 jerk_now = is_collision_impact(
 list(self._accel_x_ring),
 jerk_threshold_m_s3=self._collision_jerk,
 window_n=self._collision_window_n,
 )
 if jerk_now:
 # Mark the spike; the stall path confirms within 0.5 s.
 self._last_jerk_spike_at = now
 # Combined collision signal: jerk spike + wheels stopped soon after.
 if (
 self._last_jerk_spike_at is not None
 and (now - self._last_jerk_spike_at) <= 0.5
 and (now - self._last_collision_emit_at) >= self._collision_cooldown_s
 and self._is_currently_stalled_for_collision
 ):
 self._last_collision_emit_at = now
 self._last_jerk_spike_at = None
 self._emit_event(
 "collision_impact",
 f"jerk_spike+stall cmd_v={self._last_cmd_v_seen:.2f} "
 f"odom_v={self._last_odom_v_seen:.2f}",
 )
 self.get_logger.warn("COLLISION impact: jerk + stall confirmed")

 def _is_currently_stalled_for_collision(self) -> bool:
 """Lightweight stall check for collision conditioning — looks at
 the most recent ~10 cmd/odom samples (faster than the 1 s
 wheel-stall window so collisions can be detected promptly)."""
 if len(self._cmd_v_ring) < 5 or len(self._odom_v_ring) < 5:
 return False
 cmd_avg = sum(self._cmd_v_ring[-5:]) / 5
 odom_avg = sum(self._odom_v_ring[-5:]) / 5
 return cmd_avg > 0.03 and odom_avg < 0.02

 # ---- cmd_vel callback (20 Hz) --------------------------------------

 def _on_cmd_vel(self, msg) -> None:
 self._cmd_count += 1
 v = float(msg.twist.linear.x)
 self._last_cmd_v_seen = v

 # arm gate: arm when planner first commands motion.
 now = time.monotonic
 wants_motion = abs(v) > self._stall_cmd_threshold
 if wants_motion and not self._stall_armed:
 self._stall_armed = True
 self._stall_arm_time = now
 self._cmd_v_ring = []
 self._odom_v_ring = []
 elif not wants_motion:
 self._stall_armed = False
 self._stall_arm_time = None
 self._cmd_v_ring = []
 self._odom_v_ring = []

 if self._stall_armed:
 self._cmd_v_ring.append(v)
 if len(self._cmd_v_ring) > self._stall_n:
 self._cmd_v_ring = self._cmd_v_ring[-self._stall_n:]

 # ---- odometry callback (20+ Hz) ------------------------------------

 def _on_odom(self, msg) -> None:
 self._odom_count += 1
 v = abs(float(msg.twist.twist.linear.x))
 self._last_odom_v_seen = v
 if not self._stall_armed:
 return
 self._odom_v_ring.append(v)
 if len(self._odom_v_ring) > self._stall_n:
 self._odom_v_ring = self._odom_v_ring[-self._stall_n:]

 now = time.monotonic
 # Fast-window wheel-impact check fires at ~300 ms (6 samples
 # at 20 Hz) — catches transient hits before is_stalled has had
 # time to average over its full 1 s window.
 if is_wheel_impact(
 recent_cmd_v=self._cmd_v_ring,
 recent_odom_v=self._odom_v_ring,
 n=self._wheel_impact_n,
 cmd_threshold=self._stall_cmd_threshold,
 odom_threshold=self._stall_odom_threshold,
 ):
 if (now - self._last_wheel_impact_emit_at) >= self._wheel_impact_cooldown_s:
 self._last_wheel_impact_emit_at = now
 seg_cmd = self._cmd_v_ring[-self._wheel_impact_n:]
 seg_odom = self._odom_v_ring[-self._wheel_impact_n:]
 cmd_avg = sum(seg_cmd) / len(seg_cmd)
 odom_avg = sum(seg_odom) / len(seg_odom)
 self._emit_event(
 "wheel_impact",
 f"cmd_avg={cmd_avg:.2f} odom_avg={odom_avg:.2f}",
 )
 self.get_logger.warn(
 f"WHEEL_IMPACT: cmd_avg={cmd_avg:.2f} "
 f"odom_avg={odom_avg:.2f}",
 )

 # Stall check fires once both rings are full.
 if is_stalled(
 recent_cmd_v=self._cmd_v_ring,
 recent_odom_v=self._odom_v_ring,
 n=self._stall_n,
 cmd_threshold=self._stall_cmd_threshold,
 odom_threshold=self._stall_odom_threshold,
 ):
 if (now - self._last_stall_emit_at) >= self._stall_cooldown_s:
 self._last_stall_emit_at = now
 cmd_avg = sum(self._cmd_v_ring) / len(self._cmd_v_ring)
 odom_avg = sum(self._odom_v_ring) / len(self._odom_v_ring)
 self._emit_event(
 "wheel_stall",
 f"cmd_avg={cmd_avg:.2f} odom_avg={odom_avg:.2f}",
 )
 self.get_logger.warn(
 f"WHEEL_STALL: cmd_avg={cmd_avg:.2f} "
 f"odom_avg={odom_avg:.2f}",
 )
 # Stall-driven recovery : wheels commanded
 # forward but odom flat — something low/invisible is
 # blocking. Reuse the tilt FSM's reverse → reverse_clear
 # → spin sequence. Chassis is level, so the IMU-driven
 # exit (release_hold_s = 1s) advances to reverse_clear
 # cleanly without a separate state machine.
 if self._stall_react and self._tilt_phase == "idle":
 self._tilt_phase = "reverse"
 self._level_since = None
 self._reverse_clear_until_s = None
 self._spin_until_s = None
 self.get_logger.warn(
 "STALL → recovery (reverse + scan)",
 )

 # ---- TrialEvent emission ------------------------------------------

 def _emit_event(self, event_name: str, detail: str) -> None:
 ev = TrialEvent
 ev.stamp = self.get_clock.now.to_msg
 ev.event = event_name
 ev.detail = detail
 self.events_pub.publish(ev)

 # ---- Tilt FSM intent republisher (10 Hz while non-idle) -----------
 #
 # Phase "reverse" : TILT_REVERSE v=tilt_reverse_v (back off)
 # Phase "reverse_clear" : TILT_REVERSE same v (post-level clearance)
 # Phase "spin" : TILT_SPIN w=tilt_spin_w (change heading)
 # Phase "settle" : zero v=0 w=0 (let perception_fusion refresh)
 # Phase "wedged_announce": 360° spin at wedge_announce_w (operator signal)
 # Phase "wedged_park" : terminal STOP, beats reactive until restart
 # Phase "idle" : nothing emitted; downstream nav resumes

 def _tick_tilt_intent(self) -> None:
 if self._tilt_phase == "idle":
 return
 out = CommandIntent
 out.stamp = self.get_clock.now.to_msg
 out.source = "anomaly"
 out.priority = self._intent_priority
 out.confidence = 1.0
 if self._tilt_phase in ("reverse", "reverse_clear"):
 out.label = "TILT_REVERSE"
 out.cmd.linear.x = self._tilt_reverse_v
 out.cmd.angular.z = 0.0
 elif self._tilt_phase == "spin":
 out.label = "TILT_SPIN"
 out.cmd.linear.x = 0.0
 out.cmd.angular.z = self._tilt_spin_w
 elif self._tilt_phase == "settle":
 out.label = "TILT_SETTLE"
 out.cmd.linear.x = 0.0
 out.cmd.angular.z = 0.0
 elif self._tilt_phase == "wedged_announce":
 out.label = "WEDGED_ANNOUNCE_360"
 out.cmd.linear.x = 0.0
 out.cmd.angular.z = self._wedge_announce_w
 elif self._tilt_phase == "wedged_park":
 out.label = "WEDGED_PARK"
 out.cmd.linear.x = 0.0
 out.cmd.angular.z = 0.0
 self.intent_pub.publish(out)

 # ---- 1 Hz instrumentation -----------------------------------------

 def _tick_log(self) -> None:
 self.get_logger.info(
 f"anomaly imu={self._imu_count}/s "
 f"cmd={self._cmd_count}/s odom={self._odom_count}/s "
 f"tilt={self._tilt_phase} stall_armed={self._stall_armed} "
 f"roll={math.degrees(self._last_roll_rad):.1f}° "
 f"pitch={math.degrees(self._last_pitch_rad):.1f}°",
 )
 self._imu_count = 0
 self._cmd_count = 0
 self._odom_count = 0

 return rclpy, AnomalyDetector


def main(args=None) -> None:
 rclpy, AnomalyDetector = _build_node
 rclpy.init(args=args)
 node = AnomalyDetector
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
