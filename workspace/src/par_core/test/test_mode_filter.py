"""Pure-function tests for the ModeState gate.

The module is loaded by file path so the test runs on bare pytest without
the ROS-dependent ``par_core/__init__.py`` (safety_layer pulls in
``geometry_msgs.msg.Twist``, only present in the dev container).
"""
import importlib.util
import pathlib

import pytest

_HERE = pathlib.Path(__file__).resolve.parent
_MOD_PATH = _HERE.parent / "par_core" / "mode_filter.py"
_spec = importlib.util.spec_from_file_location("mode_filter_under_test", _MOD_PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
ModeState = _mod.ModeState
VALID_MODES = _mod.VALID_MODES
KNOWN_MODES = _mod.KNOWN_MODES


def test_default_is_active_in_bound_mode_when_no_default_passed -> None:
 """Baseline + per-scene model: a node launched into
 its scene is immediately active, because /par/active_mode no longer
 has a publisher in the per-scene runtime."""
 for mode in ("A", "B", "C", "D"):
 assert ModeState(mode).is_active is True


def test_explicit_default_mode_overrides_self_active_default -> None:
 """Legacy mode-driven runtime relied on default_mode='A' so boot-mode-A
 naturally activated only the QR node. That behaviour is still available
 by passing default_mode explicitly."""
 for mode in ("B", "C", "D"):
 assert ModeState(mode, default_mode="A").is_active is False
 assert ModeState("A", default_mode="A").is_active is True


def test_update_flips_active_when_mode_matches -> None:
 gate = ModeState("B", default_mode="A")
 assert gate.is_active is False
 changed = gate.update("B")
 assert changed is True
 assert gate.is_active is True


def test_update_flips_to_inactive_when_mode_changes_away -> None:
 gate = ModeState("B") # default-active in B
 assert gate.is_active is True
 changed = gate.update("C")
 assert changed is True
 assert gate.is_active is False


def test_update_returns_false_when_state_unchanged -> None:
 gate = ModeState("B", default_mode="A")
 assert gate.update("C") is False # was inactive, still inactive
 gate.update("B")
 assert gate.update("B") is False # was active, still active


def test_unknown_mode_is_silently_ignored -> None:
 """An ActiveMode message with an unrecognised mode keeps the gate at
 its previous state — the invariant is to never go indeterminate."""
 gate = ModeState("B")
 assert gate.update("garbage") is False
 assert gate.is_active is True
 assert gate.current_mode == "B"


def test_invalid_active_in_mode_raises -> None:
 with pytest.raises(ValueError):
 ModeState("E")


def test_invalid_default_mode_raises -> None:
 with pytest.raises(ValueError):
 ModeState("B", default_mode="?")


def test_valid_modes_is_abcd -> None:
 assert VALID_MODES == ("A", "B", "C", "D")


def test_idle_default_mode_keeps_gate_closed -> None:
 """default_mode='IDLE' is the pattern used by ND + gesture nodes so the
 gate stays closed until the supervisor (or scripts/scene.sh a/d) flips
 /par/active_mode to a real bound mode."""
 for bound in ("A", "B", "C", "D"):
 gate = ModeState(bound, default_mode="IDLE")
 assert gate.current_mode == "IDLE"
 assert gate.is_active is False


def test_idle_message_closes_an_active_gate -> None:
 """Receiving 'IDLE' on /par/active_mode must take effect — the supervisor
 publishes IDLE on entry and scripts/scene.sh idle does the same."""
 gate = ModeState("A") # default-active in A
 assert gate.is_active is True
 changed = gate.update("IDLE")
 assert changed is True
 assert gate.current_mode == "IDLE"
 assert gate.is_active is False


def test_known_modes_includes_idle -> None:
 assert KNOWN_MODES == ("A", "B", "C", "D", "IDLE")
