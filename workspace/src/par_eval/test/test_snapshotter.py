"""Unit tests for par_eval.snapshotter pure-function helpers.

The ROS-side capture pipeline (CvBridge, cv2.imwrite, OAK image cache) is
integration-tested on hardware; here we pin only the trigger predicates.filename generation, which determine which events produce captures.
"""
from __future__ import annotations

import datetime as _dt

from par_eval.snapshotter import (
 SNAPSHOT_TRIGGER_EVENTS,
 should_snapshot_event,
 should_snapshot_intent,
 snapshot_filename,
)


# --- should_snapshot_event ------------------------------------------------

def test_known_trigger_events_capture:
 for event in ("collision", "dyn_entered", "dead_end_seen",
 "manual_stop", "trapped", "qr_read", "gesture_read",
 "tilt", "collision_impact", "wheel_stall", "wheel_impact"):
 assert should_snapshot_event(event), f"{event!r} must trigger capture"


def test_unknown_events_do_not_capture:
 for event in ("stale_perception", "arbiter_switch", "spin_done", ""):
 assert not should_snapshot_event(event), f"{event!r} must not trigger capture"


def test_trigger_set_is_finite_and_documented:
 """Catch typos / accidental additions. The set should change deliberately."""
 assert SNAPSHOT_TRIGGER_EVENTS == frozenset({
 "collision", "dyn_entered", "dead_end_seen",
 "manual_stop", "trapped", "qr_read", "gesture_read",
 "tilt", "collision_impact", "wheel_stall", "wheel_impact",
 })


# --- should_snapshot_intent -----------------------------------------------

def test_dead_end_label_triggers_first_snap:
 assert should_snapshot_intent(
 "DEAD_END_WEDGE", last_snap_at=None, now_s=100.0
 )
 assert should_snapshot_intent(
 "DEAD_END_LS2", last_snap_at=None, now_s=100.0
 )
 assert should_snapshot_intent(
 "DEAD_END", last_snap_at=None, now_s=100.0 # legacy VFH+ vocabulary
 )


def test_non_dead_end_labels_do_not_trigger:
 for label in ("HSGR", "LS1", "LS2", "FORWARD", "AVOID", "STOP", ""):
 assert not should_snapshot_intent(
 label, last_snap_at=None, now_s=100.0
 )


def test_dead_end_within_debounce_window_suppressed:
 """A second DEAD_END within debounce_s of the first must not retrigger."""
 last = 100.0
 now = 102.0 # 2 s later, default debounce 5 s
 assert not should_snapshot_intent(
 "DEAD_END_WEDGE", last_snap_at=last, now_s=now
 )


def test_dead_end_after_debounce_retriggers:
 last = 100.0
 now = 106.0 # 6 s later, beyond default 5 s
 assert should_snapshot_intent(
 "DEAD_END_WEDGE", last_snap_at=last, now_s=now
 )


def test_custom_debounce_window_respected:
 last = 100.0
 now = 102.0
 # With a tighter 1.5 s debounce, 2 s gap is enough.
 assert should_snapshot_intent(
 "DEAD_END_WEDGE", last_snap_at=last, now_s=now, debounce_s=1.5
 )
 # With a wider 10 s debounce, 2 s gap is not enough.
 assert not should_snapshot_intent(
 "DEAD_END_WEDGE", last_snap_at=last, now_s=now, debounce_s=10.0
 )


# --- snapshot_filename ----------------------------------------------------

def test_filename_format_lex_sortable:
 """Filenames must lex-sort by time then sequence within the same millisecond."""
 fixed_time = _dt.datetime(2026, 5, 15, 14, 32, 45, 678000)
 fname = snapshot_filename("collision", source="reactive", seq=12, now=fixed_time)
 assert fname == "143245_678_0012_collision_reactive.jpg"


def test_filename_lex_sort_orders_by_time:
 """Earlier capture sorts before later capture."""
 early = _dt.datetime(2026, 5, 15, 14, 32, 45, 678000)
 late = _dt.datetime(2026, 5, 15, 14, 32, 45, 999000)
 f_early = snapshot_filename("a", seq=1, now=early)
 f_late = snapshot_filename("b", seq=2, now=late)
 assert sorted([f_late, f_early]) == [f_early, f_late]


def test_filename_sanitises_unsafe_characters:
 """Slashes and spaces in event/source must be replaced so the filename
 cannot escape the captures dir."""
 fname = snapshot_filename(
 "/foo bar", source="x/y", seq=1,
 now=_dt.datetime(2026, 5, 15, 14, 32, 45, 678000),
 )
 assert "/" not in fname
 assert " " not in fname
 assert "___foo_bar" in fname or "_foo_bar" in fname # whichever sanitiser catches it
 assert "x_y" in fname


def test_filename_default_source_is_dash:
 fname = snapshot_filename("collision", seq=1,
 now=_dt.datetime(2026, 5, 15, 14, 32, 45, 678000))
 assert "_-_collision" not in fname # event must come before source
 assert fname.endswith("_collision_-.jpg")
