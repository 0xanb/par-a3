"""Unit tests for the post- recovery_controller predicate widening.

The recovery FSM logic (phase transitions, timers) is integration-tested in
hardware. Here we pin only the two predicate functions that changed:

  * `is_dead_end_label` — accepts ND's DEAD_END_LS2 / DEAD_END_WEDGE alongside
    VFH+'s legacy "DEAD_END".
  * `should_early_exit_spin` — accepts ND's HSGR alongside VFH+'s "FORWARD".
"""
from __future__ import annotations

from par_reactive_nav.recovery_controller import (
    is_clean_forward,
    is_dead_end_label,
    should_early_exit_spin,
    should_escalate_trapped,
)


# --- is_dead_end_label -----------------------------------------------------

def test_dead_end_legacy_vfh_label_triggers():
    assert is_dead_end_label("DEAD_END")


def test_dead_end_nd_geometric_label_triggers():
    assert is_dead_end_label("DEAD_END_LS2")


def test_dead_end_nd_wedge_label_triggers():
    assert is_dead_end_label("DEAD_END_WEDGE")


def test_dead_end_unrelated_labels_do_not_trigger():
    for label in ("HSGR", "HSWR", "HSNR", "LS1", "LS2", "FORWARD", "AVOID", ""):
        assert not is_dead_end_label(label), f"{label!r} unexpectedly triggers recovery"


# --- should_early_exit_spin ------------------------------------------------

def test_spin_exits_on_vfh_forward_label():
    """Pre- VFH+ baseline: FORWARD with substantial v → exit."""
    assert should_early_exit_spin("spin", "FORWARD", 0.18)


def test_spin_exits_on_nd_hsgr_label():
    """Post-: HSGR is ND's equivalent of FORWARD; must also exit."""
    assert should_early_exit_spin("spin", "HSGR", 0.18)


def test_spin_does_not_exit_on_low_velocity():
    """v <= 0.10 means the planner is not actually committing to forward
    motion; do not abort recovery."""
    assert not should_early_exit_spin("spin", "FORWARD", 0.05)
    assert not should_early_exit_spin("spin", "HSGR", 0.10)  # boundary: must be strictly >


def test_spin_does_not_exit_on_avoid_or_ls_labels():
    """AVOID / LS1 / HSWR are 'steering around something' or 'low safety' —
    not a clean forward signal."""
    for label in ("AVOID", "LS1", "LS2", "HSWR", "HSNR"):
        assert not should_early_exit_spin("spin", label, 0.18), (
            f"{label!r} unexpectedly aborts recovery"
        )


def test_early_exit_only_applies_during_spin():
    """The predicate must be False outside the spin phase, regardless of label/v."""
    for phase in ("idle", "reverse"):
        assert not should_early_exit_spin(phase, "FORWARD", 0.18)
        assert not should_early_exit_spin(phase, "HSGR", 0.18)


# --- should_escalate_trapped ----------------------------------------------

def test_trap_below_threshold_does_not_escalate():
    """1 or 2 consecutive recoveries are normal; do not escalate."""
    assert not should_escalate_trapped(0)
    assert not should_escalate_trapped(1)
    assert not should_escalate_trapped(2)


def test_trap_at_threshold_escalates():
    """3rd consecutive recovery triggers the trap escalation."""
    assert should_escalate_trapped(3)
    # And subsequent counts also escalate (idempotent — TrialEvent
    # is one-shot via the controller's _trapped_emitted flag).
    assert should_escalate_trapped(4)
    assert should_escalate_trapped(10)


def test_trap_threshold_overridable():
    """Operator can tune via ROS param trap_threshold."""
    assert not should_escalate_trapped(3, threshold=5)
    assert should_escalate_trapped(5, threshold=5)


# --- is_clean_forward -----------------------------------------------------

def test_clean_forward_accepts_both_vocabularies():
    assert is_clean_forward("FORWARD", 0.18)
    assert is_clean_forward("HSGR", 0.18)


def test_clean_forward_rejects_low_velocity():
    """v <= 0.10 means planner is hesitating — not a clean signal."""
    assert not is_clean_forward("FORWARD", 0.05)
    assert not is_clean_forward("HSGR", 0.10)  # boundary: must be strictly >


def test_clean_forward_rejects_other_labels():
    for label in ("AVOID", "LS1", "LS2", "DEAD_END_WEDGE", "HSWR", "HSNR", ""):
        assert not is_clean_forward(label, 0.18), f"{label!r} unexpectedly clean"
