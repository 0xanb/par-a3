"""Contract tests for par_gesture.gesture_interpreter's intent table.

Mirrors test_interpreter_core in par_qr_nav: turns must be in-place with a
finite duration so the interpreter can auto-transition back to cruise.not circle forever.
"""
from par_gesture.interpreter_core import (
 CRUISE_V,
 TURN_W,
 UTURN_W,
 intent_for,
 is_modifier,
 stationary_intent,
)


def test_stop_is_zero_velocity_and_latched -> None:
 it = intent_for("STOP")
 assert it is not None
 assert it.linear_x == 0.0 and it.angular_z == 0.0
 assert it.duration_s is None
 # Operator-initiated STOP must beat reactive avoidance (70). See.
 assert it.priority == 85


def test_emergency_stop_has_high_priority -> None:
 it = intent_for("EMERGENCY_STOP")
 assert it is not None
 # 97 since — bumped above anomaly TILT_REVERSE (95)
 # so the operator can interrupt automated tilt recovery if reverse
 # would be dangerous (e.g. cliff behind).
 assert it.priority == 97
 assert it.linear_x == 0.0 and it.angular_z == 0.0


def test_go_and_resume_cruise_at_cruise_v -> None:
 for label in ("GO", "RESUME"):
 it = intent_for(label)
 assert it is not None
 assert it.linear_x == CRUISE_V
 assert it.duration_s is None


def test_turns_are_in_place_with_finite_duration -> None:
 for label in ("TURN_LEFT", "TURN_RIGHT", "U_TURN"):
 it = intent_for(label)
 assert it is not None, label
 assert it.linear_x == 0.0, f"{label} must not move forward while turning"
 assert it.duration_s is not None and it.duration_s > 0.0


def test_turn_yaw_signs -> None:
 assert intent_for("TURN_LEFT").angular_z == TURN_W
 assert intent_for("TURN_RIGHT").angular_z == -TURN_W
 assert intent_for("U_TURN").angular_z == UTURN_W


def test_unknown_label_returns_none -> None:
 assert intent_for("TELEPORT") is None


def test_stationary_initial_state_is_stopped -> None:
 s = stationary_intent
 assert s.linear_x == 0.0
 assert s.label == "STOP"


def test_modifier_classification -> None:
 """Aligned with par_qr_nav.MODIFIER_VERBS as of: only the
 SPEED adjustments truly require existing motion. Turns are NOT
 modifiers — the operator may rotate from rest after a STOP, with the
 spin wrapped in 0.4 s settle phases at entry and exit."""
 for label in ("SPEED_UP", "SLOW_DOWN", "SPEED_DOWN"):
 assert is_modifier(label)
 for label in (
 "EMERGENCY_STOP", "STOP", "GO", "RESUME",
 "TURN_LEFT", "TURN_RIGHT", "U_TURN",
 ):
 assert not is_modifier(label), (
 f"{label} is a state-setter (or a turn) and must be valid from "
 "a stationary robot"
 )
