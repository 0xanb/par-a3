"""Pure mapping table: QR payload -> (linear, angular, label, priority, duration).

No ROS, no msg types. Easy to unit test.

Vocabulary scope (locked): seven control verbs only — STOP, GO,
SPEED_UP, SPEED_DOWN, TURN_LEFT, TURN_RIGHT, U_TURN. The earlier
MODE_A/B/C/D mode-switch verbs and confirmation-dance machinery were removed
when the runtime reverted from always-on mode-gating to per-scene SSH
launchesmd -revised). The archived
mode-driven design lives under workspace/src/-archived/mode-driven-runtime/.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Intent:
 linear_x: float
 angular_z: float
 label: str
 priority: int = 60 # QR intents sit at priority 60 by default
 source: str = "qr"
 # If set, this intent is a finite maneuver: after ``duration_s`` seconds the
 # command interpreter auto-transitions to the cruise (GO) intent. If None,
 # the intent is latched indefinitely until a new verb arrives.
 duration_s: float | None = None


CRUISE_V: float = 0.20 # m/s — matches par_core.SafetyConfig.v_max ladder
TURN_W: float = 0.8 # rad/s for 90° turns
UTURN_W: float = 1.2 # rad/s for a 180° U-turn

# Empirical friction offsets added to each rotation maneuver. With the
# stop-and-turn pattern in command_interpreter, the rotation starts from
# a true zero (after SETTLE_BEFORE) and ends back at zero (during
# SETTLE_AFTER). In an ideal frictionless model the accel and decel
# ramps cancel and total rotation = w × duration_s. In practice motor
# stator friction and gear losses cause the actual rotation to undershoot
# by a small constant. Observed empirically at v_max=0.20, w_max=1.20
# on hard floor with a charged battery: TURN under by ~4°, U_TURN under
# by ~10°. These offsets extend the maneuver duration to compensate.
_TURN_FRICTION_OFFSET_RAD: float = math.radians(4)
_UTURN_FRICTION_OFFSET_RAD: float = math.radians(10)


def _maneuver_duration(target_rad: float, w_target: float, friction_offset_rad: float) -> float:
 return (target_rad + friction_offset_rad) / w_target


_TURN_90_S: float = _maneuver_duration(math.pi / 2, TURN_W, _TURN_FRICTION_OFFSET_RAD) # ≈ 2.05 s
_UTURN_180_S: float = _maneuver_duration(math.pi, UTURN_W, _UTURN_FRICTION_OFFSET_RAD) # ≈ 2.76 s


# Explicit operator STOP (QR card) must beat reactive avoidance (70),
# anomaly TILT recovery (95), recovery_controller high (90) — but must
# STILL LOSE to gesture EMERGENCY_STOP (97, closed fist) so the fist
# remains the absolute kill above every operator-issued verb. Raised
# 85 → 96 (follow-up): the operator standing next to
# the chassis with a STOP card needs to suppress tilt-FSM auto-rotation
# when the IMU reads a sustained tilt from a real but operator-known
# surface irregularity (carpet edge, cable, baseboard lip). Tradeoff:
# anomaly cannot auto-rescue once STOP is latched — operator physically
# intervenes. Acceptable on demo day where the operator is right there.
STOP_PRIORITY: int = 96


def verb_to_intent(
 verb: str,
 restrict_to: frozenset[str] | None = None,
) -> Intent | None:
 """Map a QR verb to its Intent, with optional vocabulary restriction.

 When ``restrict_to`` is None (default) all seven verbs resolve normally. unknown verbs return None — preserves the original contract.

 When ``restrict_to`` is a set of allowed verbs (e.g. ``{"STOP","GO"}`` for
 the 2-mode pivot mode A), any verb not in the set is treated as rejected
 and returns ``recovering_intent`` — the same one-shot RECOVERING event
 used today for unknown payloads. The caller does not need to special-case
 rejected verbs; the publisher path is identical to the unknown-payload
 path. The input ``restrict_to`` is never mutated.
 """
 table: dict[str, Intent] = {
 "STOP": Intent(0.0, 0.0, "STOP", priority=STOP_PRIORITY),
 "GO": Intent(CRUISE_V, 0.0, "GO"),
 "SPEED_UP": Intent(1.5 * CRUISE_V, 0.0, "SPEED_UP"),
 "SPEED_DOWN": Intent(0.5 * CRUISE_V, 0.0, "SPEED_DOWN"),
 # Turns are in-place, time-bounded, then auto-resume the last cruise
 # intent (not a hardcoded GO — see command_interpreter). Doing the
 # rotation in place keeps the arc predictable on a 0.5 m wide corridor.
 "TURN_LEFT": Intent(0.0, TURN_W, "TURN_LEFT", duration_s=_TURN_90_S),
 "TURN_RIGHT": Intent(0.0, -TURN_W, "TURN_RIGHT", duration_s=_TURN_90_S),
 "U_TURN": Intent(0.0, UTURN_W, "U_TURN", duration_s=_UTURN_180_S),
 }
 if restrict_to is not None and verb in table and verb not in restrict_to:
 return recovering_intent
 return table.get(verb)


# Verbs that REQUIRE the robot to already be moving to take effect.
# SPEED_UP/SPEED_DOWN truly modify a cruise speed — they only make sense
# when something is already moving. TURN_LEFT, TURN_RIGHT, and U_TURN are
# NOT modifiers: the operator can rotate from rest (after a STOP) without
# first re-issuing GO.
MODIFIER_VERBS: frozenset[str] = frozenset({
 "SPEED_UP", "SPEED_DOWN",
})


def is_modifier(verb: str) -> bool:
 return verb in MODIFIER_VERBS


def stationary_intent -> Intent:
 """Default resume target at boot — robot is stopped. GO is the only verb
 that can break this state; modifiers (turns, speed adjustments) are
 rejected while the robot is stationary."""
 return Intent(0.0, 0.0, "STOP", priority=STOP_PRIORITY)


# RECOVERING priority is deliberately the lowest possible value so that the
# recovering event coexists with any fresh real intent without overriding it.
#md — RECOVERING is shaped as a one-shot
# event, not a persistent FSM state, so it must yield to anything else.
RECOVERING_PRIORITY: int = 0


def recovering_intent -> Intent:
 """One-shot event-shaped intent emitted when the QR detector decodes an
 unknown payload. Zero velocity, lowest priority — the arbiter still picks
 a higher-priority intent if one exists. Visible on /par/events for the
 operator to debug.md."""
 return Intent(0.0, 0.0, "recovering", priority=RECOVERING_PRIORITY, source="qr")


def cruise_intent -> Intent:
 """Fallback resume target when this channel has not seen its own cruise
 but another channel is driving the robot (cross-channel motion)."""
 return Intent(CRUISE_V, 0.0, "GO")


def any_source_moving(latest: dict[str, tuple[float, float]],
 now_s: float,
 fresh_s: float = 0.5) -> bool:
 """True when any source has published a non-zero-velocity intent within
 ``fresh_s`` seconds of ``now_s``. ``latest`` maps each source name to its
 most recent (monotonic_time, linear_x) pair.

 Matches the arbiter's ``stale_after_s=0.5`` drop window so interpreters. arbiter agree on what 'currently moving' means."""
 return any(v > 0.0 and now_s - t < fresh_s for t, v in latest.values)


def update_motion_record(latest: dict[str, tuple[float, float]],
 source: str,
 linear_x: float,
 now_s: float,
 self_source: str = "qr") -> None:
 """Record an observed CommandIntent in ``latest`` for cross-channel motion
 tracking. Mutates ``latest`` in place.

 Cross-channel sources (anything except ``self_source``): always record. A
 fresh zero from another channel is meaningful — it means that source is
 not currently asking the robot to move.

 Self-source: only record when ``linear_x > 0`` — i.e. when we actually
 publish a positive cruise. The interpreter republishes ``0.0`` every tick
 during the SETTLE_BEFORE / SETTLE_AFTER phases of a stop-and-turn maneuver,
 and without this guard each settle tick would overwrite the most recent
 positive cruise velocity, pulling ``any_source_moving`` to False. Any
 SPEED_UP / SPEED_DOWN card flashed during the rotation would then drop
 with ``"no source is moving"`` even though a valid GO had just been
 issued. See command_interpreter.py:_on_intent for the call site.
 """
 if source == self_source and linear_x <= 0.0:
 return
 latest[source] = (now_s, linear_x)
