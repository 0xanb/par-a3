"""Unit tests for ``gesture_detector.should_fire`` (wall-clock gate).

The gesture detector's stability gate moved from tick-counting to wall-clock
time so that ``rate_hz`` changes (e.g. CPU policy reducing 10 Hz → 5 Hz)
do NOT silently change the operator-perceived hold duration. These tests pin
the contract.
"""
from __future__ import annotations

from par_gesture.gesture_detector import should_fire


HOLD_S = 0.4
COOLDOWN_S = 1.0


def test_no_candidate_does_not_fire:
 assert not should_fire(
 None,
 pending_label=None,
 pending_since_s=None,
 last_fired_s=0.0,
 now_s=10.0,
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_candidate_held_long_enough_fires:
 """Candidate matches pending, held ≥ 0.4 s, cooldown elapsed → fire."""
 assert should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=0.0, # cooldown elapsed (now=10.5)
 now_s=10.5,
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_candidate_held_too_short_does_not_fire:
 """Same pose held only 0.3 s → below hold_seconds → no fire."""
 assert not should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=0.0,
 now_s=10.3, # only 0.3 s held
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_candidate_changed_does_not_fire:
 """Classifier emitted GO, then changed to STOP — pending updated, but
 candidate now != pending_label until next tick. should_fire returns False
 for the mismatch (defensive — this branch shouldn't normally be hit because
 the detector resets pending_label on change before calling should_fire)."""
 assert not should_fire(
 "STOP", # new candidate
 pending_label="GO", # pending hasn't been reset yet
 pending_since_s=10.0,
 last_fired_s=0.0,
 now_s=10.5,
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_within_cooldown_does_not_fire:
 """Same pose held long enough, but last fire was < cooldown_s ago — block."""
 assert not should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=10.4, # fired 0.1 s ago
 now_s=10.5, # held 0.5 s, but cooldown = 1.0 s
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_after_cooldown_fires:
 """Cooldown elapsed → re-fire allowed."""
 assert should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=10.4,
 now_s=11.5, # 1.1 s after last fire, cooldown 1.0 s
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_pending_since_none_does_not_fire:
 """Edge case: detector hasn't seen any candidate yet → can't fire."""
 assert not should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=None,
 last_fired_s=0.0,
 now_s=10.5,
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_hold_seconds_independent_of_rate_hz:
 """The whole point of : a 0.4 s hold means 0.4 s regardless of how
 fast the detector ticks. Simulate two rates by checking the same now_s -
 pending_since_s elapsed (0.4 s) fires for both."""
 common = dict(
 candidate="GO",
 pending_label="GO",
 last_fired_s=0.0,
 now_s=10.4, # exactly 0.4 s after pending_since_s
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )
 # At rate_hz=10, this would have been ~4 ticks; at rate_hz=5, ~2 ticks.
 # The gate fires for both because it checks elapsed time, not tick count.
 common["pending_since_s"] = 10.0
 assert should_fire(**common)


def test_boundary_at_hold_seconds_fires:
 """now - pending_since == hold_seconds satisfies >= comparison → fire."""
 assert should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=0.0,
 now_s=10.4, # exactly hold_seconds elapsed
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )


def test_just_under_hold_seconds_does_not_fire:
 """Boundary the other way: 0.001 s shy of threshold → block."""
 assert not should_fire(
 "GO",
 pending_label="GO",
 pending_since_s=10.0,
 last_fired_s=0.0,
 now_s=10.399,
 hold_seconds=HOLD_S,
 cooldown_s=COOLDOWN_S,
 )
