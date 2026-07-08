"""ModeState — pure helper for behaviour-node mode gating.

Each non-QR behaviour node owns one ModeState instance, subscribes to
``/par/active_mode``, and updates the state from each incoming message. The
node's publish path checks ``is_active()`` and skips heavy work / publish
when its mode is not the current active mode.

Design choice:
    Pure state held in this class so the gate is unit-testable without ROS.
    The ROS subscription stays in the node (one-line callback into ``update``).

 -revised for why mode-gating was kept
in code but the always-on runtime that drove ``/par/active_mode`` was
retired in favour of per-scene SSH launches .
"""
from __future__ import annotations


VALID_MODES: tuple[str, ...] = ("A", "B", "C", "D")
# Values that can appear on /par/active_mode. "IDLE" is a sentinel
# published by the supervisor / scripts/scene.sh idle to keep every
# behaviour gate closed; no node binds to it as active_in_mode.
KNOWN_MODES: tuple[str, ...] = VALID_MODES + ("IDLE",)


class ModeState:
    """Tracks the active mode and answers ``is_active()`` for one bound mode.

 Behaviour at boot (post- baseline + per-scene revert): if no
    explicit ``default_mode`` is passed, ``current_mode`` defaults to the
    node's own ``active_in_mode``. This means a node that is explicitly
    launched (via ``./scripts/scene.sh <x>`` SSHing to ``project_<x>.launch.py``)
    is immediately active even though nobody is publishing ``/par/active_mode``
    in the per-scene runtime. If a future revival of the always-on runtime
    starts publishing ``/par/active_mode`` again, the gating still works as
 the original design intended.

    The pre-revert behaviour ``default_mode="A"`` is still available by
    passing it explicitly; this is what the boot-mode-A invariant of the
    archived ``mode-driven-runtime/command_interpreter_with_dance.py``
    relied on. ``default_mode="IDLE"`` is the 2-mode-pivot pattern: keep
    the gate closed at boot until the supervisor (or scripts/scene.sh a/d)
    flips it on.
    """

    def __init__(self, active_in_mode: str, default_mode: str | None = None) -> None:
        if active_in_mode not in VALID_MODES:
            raise ValueError(
                f"active_in_mode must be one of {VALID_MODES}, got {active_in_mode!r}"
            )
        # If no default is passed, treat the node as active in its bound
        # mode. Per-scene launches do not publish /par/active_mode, so a
        # default of "A" (the old behaviour) would leave non-A nodes
        # permanently inactive even after their scene was launched.
        if default_mode is None:
            default_mode = active_in_mode
        if default_mode not in KNOWN_MODES:
            raise ValueError(
                f"default_mode must be one of {KNOWN_MODES}, got {default_mode!r}"
            )
        self.active_in_mode: str = active_in_mode
        self.current_mode: str = default_mode

    def update(self, mode: str) -> bool:
        """Apply a new active mode. Returns True if the gate state changed
        (i.e. is_active() flipped). Unknown modes are silently ignored —
        the gate keeps its previous state rather than going indeterminate.
        """
        if mode not in KNOWN_MODES:
            return False
        was_active = self.is_active()
        self.current_mode = mode
        return self.is_active() != was_active

    def is_active(self) -> bool:
        """True iff the current active mode equals the mode this gate
        cares about."""
        return self.current_mode == self.active_in_mode
