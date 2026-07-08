"""Tests for the RECOVERING event-shaped intent ( in ).

The RECOVERING behaviour is emitted as a one-shot CommandIntent with priority
0 when an unknown QR verb is decoded. Pure-function shape tests are below;
the wiring through ``CommandInterpreter._on_detect`` is exercised on the
robot via /par/intents inspection.
"""
from par_qr_nav.interpreter_core import (
    RECOVERING_PRIORITY,
    STOP_PRIORITY,
    recovering_intent,
    verb_to_intent,
)


def test_recovering_intent_zero_velocity() -> None:
    rec = recovering_intent()
    assert rec.linear_x == 0.0
    assert rec.angular_z == 0.0


def test_recovering_intent_label_is_lowercase() -> None:
    """Lowercase label distinguishes the event-shaped intent from the
    seven SCREAMING_CASE verbs in the QR vocabulary."""
    assert recovering_intent().label == "recovering"


def test_recovering_priority_is_lowest() -> None:
    """RECOVERING must yield to any other source. Priority 0 is below the
    suggested baseline (default=10) so it cannot accidentally win an
    arbitration tie."""
    assert RECOVERING_PRIORITY == 0
    assert recovering_intent().priority == RECOVERING_PRIORITY
    assert recovering_intent().priority < STOP_PRIORITY


def test_recovering_source_is_qr() -> None:
    """The intent carries the QR source so /par/events traces back to the
    detector that produced the unknown payload."""
    assert recovering_intent().source == "qr"


def test_recovering_is_not_a_known_verb() -> None:
    """The RECOVERING label is reserved — no verb in the vocabulary maps to
    it. A QR card encoding the literal string 'recovering' should still
    return None from verb_to_intent (treated as unknown by the detector)."""
    assert verb_to_intent("recovering") is None
    assert verb_to_intent("RECOVERING") is None


def test_recovering_intent_is_not_timed() -> None:
    """RECOVERING is event-shaped, not a maneuver — no duration_s, no
    auto-resume. A new fresh intent on /par/intents simply outranks it
    via the arbiter's freshness decay."""
    assert recovering_intent().duration_s is None
