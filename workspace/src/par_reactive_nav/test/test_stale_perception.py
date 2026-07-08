"""Pure-function tests for vfh stale-perception watchdog ( lens).

The wiring through ``VFHPlanner._tick_stale_watchdog`` is exercised on the
robot via /par/events inspection. These tests cover the decision rule.
"""
from par_reactive_nav.vfh_core import should_emit_stale_event


def test_no_emission_on_boot_when_fresh() -> None:
    """Boot sets last_hist_at = now; the freshness window has not lapsed yet."""
    assert (
        should_emit_stale_event(last_hist_at=100.0, now=100.3, stale_threshold_s=0.5)
        is False
    )


def test_no_emission_when_unprimed() -> None:
    """Defensive: a callsite that forgets to prime last_hist_at should not
    spam events."""
    assert should_emit_stale_event(last_hist_at=None, now=100.0) is False


def test_emit_when_threshold_lapses() -> None:
    """No fresh histogram in the threshold window; first emission fires."""
    assert (
        should_emit_stale_event(
            last_hist_at=100.0, now=100.6, stale_threshold_s=0.5, last_emit_at=None
        )
        is True
    )


def test_debounce_suppresses_repeat_within_window() -> None:
    """A second tick during a long outage should not re-emit until the
    debounce window expires."""
    assert (
        should_emit_stale_event(
            last_hist_at=100.0,
            now=100.8,
            stale_threshold_s=0.5,
            last_emit_at=100.6,
            debounce_s=1.0,
        )
        is False
    )


def test_debounce_lifts_after_window() -> None:
    """During an outage longer than debounce, a second event should fire so
    the operator gets a refreshed reminder."""
    assert (
        should_emit_stale_event(
            last_hist_at=100.0,
            now=102.0,
            stale_threshold_s=0.5,
            last_emit_at=100.6,
            debounce_s=1.0,
        )
        is True
    )


def test_threshold_boundary_is_not_stale() -> None:
    """now - last_hist_at == threshold is fresh, not stale (closed inequality)."""
    assert (
        should_emit_stale_event(last_hist_at=100.0, now=100.5, stale_threshold_s=0.5)
        is False
    )
