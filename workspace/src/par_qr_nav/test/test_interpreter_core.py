from par_qr_nav.interpreter_core import (
    CRUISE_V,
    STOP_PRIORITY,
    TURN_W,
    UTURN_W,
    any_source_moving,
    is_modifier,
    stationary_intent,
    update_motion_record,
    verb_to_intent,
)


def test_stop_emits_zero_velocity() -> None:
    it = verb_to_intent("STOP")
    assert it is not None
    assert it.linear_x == 0.0
    assert it.angular_z == 0.0
    # Explicit operator STOP must beat reactive avoidance (70). Raised to 96
    # per so it also suppresses tilt-FSM auto-rotation; track the constant.
    assert it.priority == STOP_PRIORITY
    assert it.source == "qr"
    assert it.label == "STOP"


def test_go_emits_cruise_velocity() -> None:
    it = verb_to_intent("GO")
    assert it is not None
    assert it.linear_x == CRUISE_V
    assert it.angular_z == 0.0


def test_turn_left_is_positive_yaw() -> None:
    it = verb_to_intent("TURN_LEFT")
    assert it is not None
    assert it.angular_z > 0
    assert abs(it.angular_z - TURN_W) < 1e-9


def test_turn_right_is_negative_yaw() -> None:
    it = verb_to_intent("TURN_RIGHT")
    assert it is not None
    assert it.angular_z < 0
    assert abs(it.angular_z - (-TURN_W)) < 1e-9


def test_u_turn_has_larger_yaw_than_90_turn() -> None:
    u = verb_to_intent("U_TURN")
    t = verb_to_intent("TURN_LEFT")
    assert u is not None and t is not None
    assert abs(u.angular_z) > abs(t.angular_z)
    assert u.linear_x == 0.0
    assert abs(u.angular_z - UTURN_W) < 1e-9


def test_speed_up_and_down_bracket_cruise() -> None:
    up = verb_to_intent("SPEED_UP")
    down = verb_to_intent("SPEED_DOWN")
    assert up is not None and down is not None
    assert up.linear_x > CRUISE_V > down.linear_x > 0


def test_unknown_verb_returns_none() -> None:
    assert verb_to_intent("TELEPORT") is None


def test_cruise_and_stop_are_latched_not_timed() -> None:
    for verb in ("STOP", "GO", "SPEED_UP", "SPEED_DOWN"):
        it = verb_to_intent(verb)
        assert it is not None
        assert it.duration_s is None, f"{verb} should latch indefinitely"


def test_turns_are_in_place_with_finite_duration() -> None:
    for verb in ("TURN_LEFT", "TURN_RIGHT", "U_TURN"):
        it = verb_to_intent(verb)
        assert it is not None
        assert it.linear_x == 0.0, f"{verb} must be in-place (no forward motion)"
        assert it.duration_s is not None and it.duration_s > 0.0


def test_stop_priority_beats_reactive_avoidance() -> None:
    """Operator STOP must outrank reactive avoidance (priority 70)."""
    stop = verb_to_intent("STOP")
    assert stop is not None
    assert stop.priority >= 75, (
        "STOP priority must be strictly above reactive (70) so the operator "
        "can always halt the robot even during evasive manoeuvring."
    )


def test_any_source_moving_is_false_when_empty() -> None:
    assert any_source_moving({}, now_s=100.0) is False


def test_any_source_moving_true_for_fresh_nonzero_intent() -> None:
    latest = {"gesture": (99.8, 0.20)}  # 0.2 s old, moving
    assert any_source_moving(latest, now_s=100.0, fresh_s=0.5) is True


def test_any_source_moving_false_for_stale_intent() -> None:
    latest = {"gesture": (98.0, 0.20)}  # 2 s old, past the fresh window
    assert any_source_moving(latest, now_s=100.0, fresh_s=0.5) is False


def test_any_source_moving_false_for_fresh_zero_intent() -> None:
    latest = {"qr": (100.0, 0.0)}  # fresh STOP
    assert any_source_moving(latest, now_s=100.0, fresh_s=0.5) is False


def test_any_source_moving_is_any_not_all() -> None:
    """One fresh moving source is enough, even if others are stopped or stale."""
    latest = {
        "qr": (100.0, 0.0),       # fresh stop
        "gesture": (99.9, 0.20),  # fresh go — this one wins
        "voice": (90.0, 0.20),    # stale
    }
    assert any_source_moving(latest, now_s=100.0, fresh_s=0.5) is True


def test_stationary_intent_is_stopped() -> None:
    s = stationary_intent()
    assert s.linear_x == 0.0 and s.angular_z == 0.0
    assert s.label == "STOP"


def test_modifiers_are_speed_only() -> None:
    """SPEED_UP/SPEED_DOWN require existing motion. TURN_*/U_TURN can fire
    from rest so the operator can do STOP -> TURN_LEFT without first GO."""
    for v in ("SPEED_UP", "SPEED_DOWN"):
        assert is_modifier(v), f"{v} should be a modifier (requires motion)"
    for v in ("STOP", "GO", "TURN_LEFT", "TURN_RIGHT", "U_TURN"):
        assert not is_modifier(v), f"{v} must fire from rest"


def test_update_motion_record_records_cross_channel_zero() -> None:
    """A zero from a non-self source IS recorded — it represents a real
    'this channel has stopped' state and is meaningful for the arbiter."""
    latest: dict[str, tuple[float, float]] = {}
    update_motion_record(latest, "gesture", 0.0, now_s=10.0)
    assert latest == {"gesture": (10.0, 0.0)}


def test_update_motion_record_records_self_positive() -> None:
    """A positive linear from the self channel ('qr') IS recorded — this is
    the cruise GO that lets modifier verbs like SPEED_UP take effect."""
    latest: dict[str, tuple[float, float]] = {}
    update_motion_record(latest, "qr", CRUISE_V, now_s=10.0)
    assert latest == {"qr": (10.0, CRUISE_V)}


def test_update_motion_record_skips_self_zero() -> None:
    """REGRESSION: a self-published zero (the SETTLE_BEFORE/SETTLE_AFTER
    republish during a turn maneuver) must NOT overwrite the existing
    record. Without this guard, every settle tick clobbers the recent
    positive cruise velocity, pulling any_source_moving() to False."""
    latest = {"qr": (10.0, CRUISE_V)}
    update_motion_record(latest, "qr", 0.0, now_s=10.3)  # mid-SETTLE_BEFORE
    update_motion_record(latest, "qr", 0.0, now_s=10.6)  # mid-SETTLE_BEFORE
    assert latest == {"qr": (10.0, CRUISE_V)}, (
        "self zero must not overwrite a recent positive cruise velocity"
    )


def test_modifier_gate_survives_settle_phase() -> None:
    """End-to-end shape: GO at t=10 latches positive cruise; settle ticks at
    t=10.3 and t=10.6 must NOT pull any_source_moving to False; SPEED_UP
    flashed at t=10.4 (mid-settle, within fresh window) is accepted because
    the modifier gate still sees the cruise as fresh."""
    latest: dict[str, tuple[float, float]] = {}
    # operator shows GO
    update_motion_record(latest, "qr", CRUISE_V, now_s=10.0)
    # interpreter enters SETTLE_BEFORE for a TURN_LEFT, republishes zero
    update_motion_record(latest, "qr", 0.0, now_s=10.3)
    # operator flashes SPEED_UP mid-settle
    assert any_source_moving(latest, now_s=10.4, fresh_s=0.5) is True, (
        "modifier gate must survive settle: cruise was fresh at t=10.0, "
        "settle-zero at t=10.3 must not have overwritten it"
    )


def test_uturn_takes_longer_than_a_90_turn() -> None:
    """U-turn covers a larger angle and at a higher angular rate, so it is
    longer in wall time than a 90° turn even after the deceleration-ramp
    trim is applied."""
    t = verb_to_intent("TURN_LEFT")
    u = verb_to_intent("U_TURN")
    assert t is not None and u is not None
    assert t.duration_s is not None and u.duration_s is not None
    assert u.duration_s > t.duration_s


# ---------------------------------------------------------------------------
# Mode A vocabulary restriction (T-Dev-3, 2-mode pivot).
#
# verb_to_intent gains an optional `restrict_to` set. When set, any verb that
# is NOT in the set is coerced to the RECOVERING one-shot, matching the same
# code path used for unknown payloads. When None (the default), behaviour is
# unchanged from the original 7-verb vocabulary.
# ---------------------------------------------------------------------------


def test_verb_to_intent_unrestricted_passes_all_verbs() -> None:
    """Default (restrict_to=None) preserves the original behaviour for every
    in-vocabulary verb. Sanity check that the new keyword argument did not
    regress the unrestricted path."""
    stop = verb_to_intent("STOP")
    go = verb_to_intent("GO")
    tl = verb_to_intent("TURN_LEFT")
    assert stop is not None and stop.label == "STOP"
    assert go is not None and go.label == "GO" and go.linear_x == CRUISE_V
    assert tl is not None and tl.label == "TURN_LEFT" and tl.angular_z > 0


def test_verb_to_intent_restrict_to_stop_go_passes_stop() -> None:
    """STOP is in the restricted vocabulary, so it resolves to the normal
    STOP intent (STOP_PRIORITY, not the RECOVERING priority 0)."""
    allowed = frozenset({"STOP", "GO"})
    it = verb_to_intent("STOP", restrict_to=allowed)
    assert it is not None
    assert it.label == "STOP"
    assert it.linear_x == 0.0 and it.angular_z == 0.0
    assert it.priority == STOP_PRIORITY


def test_verb_to_intent_restrict_to_stop_go_passes_go() -> None:
    """GO is in the restricted vocabulary, so it resolves to the normal
    cruise intent."""
    allowed = frozenset({"STOP", "GO"})
    it = verb_to_intent("GO", restrict_to=allowed)
    assert it is not None
    assert it.label == "GO"
    assert it.linear_x == CRUISE_V
    assert it.angular_z == 0.0


def test_verb_to_intent_restrict_to_stop_go_rejects_turn_left() -> None:
    """TURN_LEFT is a known verb but outside the mode-A vocabulary, so it
    must come back as the RECOVERING one-shot — same shape as an unknown
    payload, so the caller does not need a separate code path."""
    allowed = frozenset({"STOP", "GO"})
    it = verb_to_intent("TURN_LEFT", restrict_to=allowed)
    assert it is not None
    assert it.label == "recovering"
    assert it.priority == 0
    assert it.linear_x == 0.0 and it.angular_z == 0.0
    assert it.source == "qr"


def test_verb_to_intent_restrict_to_stop_go_rejects_speed_up() -> None:
    """SPEED_UP is also outside the mode-A vocabulary and is rejected the
    same way TURN_LEFT is — by RECOVERING, not by None."""
    allowed = frozenset({"STOP", "GO"})
    it = verb_to_intent("SPEED_UP", restrict_to=allowed)
    assert it is not None
    assert it.label == "recovering"
    assert it.priority == 0


def test_verb_to_intent_restrict_to_does_not_mutate_input_set() -> None:
    """The restriction argument must be treated as read-only — neither a
    frozenset nor a regular set passed in may be mutated by the call."""
    immutable = frozenset({"STOP", "GO"})
    immutable_snapshot = frozenset(immutable)
    verb_to_intent("STOP", restrict_to=immutable)
    verb_to_intent("TURN_LEFT", restrict_to=immutable)
    verb_to_intent("UNKNOWN", restrict_to=immutable)
    assert immutable == immutable_snapshot

    mutable = {"STOP", "GO"}
    mutable_snapshot = set(mutable)
    # frozenset(mutable) is what command_interpreter actually passes; verify
    # that even constructing a frozenset from it does not surface a change.
    verb_to_intent("STOP", restrict_to=frozenset(mutable))
    verb_to_intent("TURN_RIGHT", restrict_to=frozenset(mutable))
    assert mutable == mutable_snapshot


def test_verb_to_intent_unknown_verb_with_restriction_still_returns_none() -> None:
    """An unknown verb (not in the master table) is still None even under
    restriction — the restriction only affects KNOWN verbs that are not in
    the allowed set. Unknown payloads keep their original semantics so the
    caller's existing 'intent is None' branch still fires."""
    allowed = frozenset({"STOP", "GO"})
    assert verb_to_intent("TELEPORT", restrict_to=allowed) is None


def test_verb_to_intent_empty_restriction_rejects_every_known_verb() -> None:
    """Passing an empty frozenset means the operator wants to reject every
    known verb — the only QR feedback the robot gives is RECOVERING events.
    Edge case worth pinning so a future operator does not get a surprise."""
    empty: frozenset[str] = frozenset()
    for verb in ("STOP", "GO", "TURN_LEFT", "TURN_RIGHT",
                 "U_TURN", "SPEED_UP", "SPEED_DOWN"):
        it = verb_to_intent(verb, restrict_to=empty)
        assert it is not None and it.label == "recovering", (
            f"empty restriction must reject known verb {verb}"
        )
