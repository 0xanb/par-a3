"""Pure-function gesture-label -> Intent table.

No ROS. Node wrapper lives in :mod:`par_gesture.gesture_interpreter`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


# Gesture cruise + turn rates. Reduced from the QR-matched
# defaults (0.20 / 0.8 / 1.2) so the operator has more reaction time when
# driving the robot reactively from hand gestures. Live testing showed the
# QR-matched speeds were too fast to interleave with hold + cooldown gates;
# the robot would have already overshot before the operator could chain the
# next command. The QR interpreter keeps its original defaults — only
# gesture-driven motion is throttled.
CRUISE_V: float = 0.12
TURN_W: float = 0.6
UTURN_W: float = 0.9
_TURN_90_S: float = (math.pi / 2) / TURN_W # ≈ 2.62 s (was 1.96 s)
_UTURN_180_S: float = math.pi / UTURN_W # ≈ 3.49 s (was 2.62 s)


@dataclass(frozen=True)
class GestureIntent:
 linear_x: float
 angular_z: float
 priority: int
 label: str
 # None = latched indefinitely; otherwise maneuver duration in seconds.
 duration_s: float | None = None


# Closed fist (EMERGENCY_STOP) is the highest-priority user action at 97 —
# above anomaly TILT_REVERSE (95) so the operator can always halt automated
# tilt recovery if the chassis is in a position where reverse would be
# dangerous (about to fall off an edge backwards, etc.). Open-palm STOP is
# a "polite stop" but must still beat reactive avoidance (70) and explicit
# QR/voice STOP (85) for operator-initiated halts.
_TABLE: dict[str, GestureIntent] = {
 "EMERGENCY_STOP": GestureIntent(0.0, 0.0, 97, "EMERGENCY_STOP"),
 "STOP": GestureIntent(0.0, 0.0, 85, "STOP"),
 "GO": GestureIntent(CRUISE_V, 0.0, 50, "GO"),
 "RESUME": GestureIntent(CRUISE_V, 0.0, 50, "RESUME"),
 "SPEED_UP": GestureIntent(1.5 * CRUISE_V, 0.0, 50, "SPEED_UP"),
 # SLOW_DOWN and SPEED_DOWN are aliases — the hand-gesture classifier
 # emits SPEED_DOWN to match the QR vocabulary 1:1 for the cross-modality
 # ablation; older callers may still emit SLOW_DOWN.
 "SLOW_DOWN": GestureIntent(0.5 * CRUISE_V, 0.0, 50, "SLOW_DOWN"),
 "SPEED_DOWN": GestureIntent(0.5 * CRUISE_V, 0.0, 50, "SPEED_DOWN"),
 "TURN_LEFT": GestureIntent(0.0, TURN_W, 50, "TURN_LEFT",
 duration_s=_TURN_90_S),
 "TURN_RIGHT": GestureIntent(0.0, -TURN_W, 50, "TURN_RIGHT",
 duration_s=_TURN_90_S),
 "U_TURN": GestureIntent(0.0, UTURN_W, 50, "U_TURN",
 duration_s=_UTURN_180_S),
}


def intent_for(label: str) -> GestureIntent | None:
 return _TABLE.get(label)


# Labels that require the robot to be moving before they take effect.
# Aligned with par_qr_nav.interpreter_core.MODIFIER_VERBS as of:
# only SPEED adjustments truly need motion. Turns are NOT modifiers — the
# operator may rotate from rest (after a STOP) without first re-issuing GO.
# The stop-then-spin-then-resume semantic in command_interpreter.py is what
# makes this safe; the maneuver wraps the spin in 0.4 s settle phases at
# entry and exit, so the robot is fully stationary before and after.
MODIFIER_LABELS: frozenset[str] = frozenset({
 "SPEED_UP", "SLOW_DOWN", "SPEED_DOWN",
})


def is_modifier(label: str) -> bool:
 return label in MODIFIER_LABELS


def stationary_intent -> GestureIntent:
 """Default resume target at boot — robot is stopped."""
 return _TABLE["STOP"]


def cruise_intent -> GestureIntent:
 """Fallback resume target when this channel has not seen its own cruise
 but another channel is driving the robot (cross-channel motion)."""
 return _TABLE["GO"]


def any_source_moving(latest: dict[str, tuple[float, float]],
 now_s: float,
 fresh_s: float = 0.5) -> bool:
 """True when any source has published a non-zero-velocity intent within
 ``fresh_s`` seconds of ``now_s``. See par_qr_nav.interpreter_core for the
 rationale. Matches arbiter stale_after_s=0.5."""
 return any(v > 0.0 and now_s - t < fresh_s for t, v in latest.values)
