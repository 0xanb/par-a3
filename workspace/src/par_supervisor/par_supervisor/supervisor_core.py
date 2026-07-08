"""Pure-function core for the simplified supervisor (post pivot).

State machine:
    BOOT -> SELF_VALIDATE -> READY_ANNOUNCE -> IDLE
    SELF_VALIDATE -> VALIDATE_FAIL on timeout (terminal)

The supervisor's only job is to wait for required topics, perform a single
360 announce spin, and latch /par/active_mode = "IDLE" once. After that the
supervisor stays alive but does nothing; mode switching to A or B is handled
by scripts/scene.sh which directly publishes /par/active_mode (see in
 for the firmware-mismatch context). The /buttons input and
/leds output paths from were dropped because the rosbot_ros snap on
this robot exposes neither topic.

No ROS imports. Exercised by host pytest. The ROS wrapper builds a
SupervisorFSM, feeds it sensor freshness via tick(), and forwards Decision
side effects to publishers.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SupervisorState(Enum):
    BOOT = "BOOT"
    SELF_VALIDATE = "SELF_VALIDATE"
    READY_ANNOUNCE = "READY_ANNOUNCE"
    IDLE = "IDLE"
    VALIDATE_FAIL = "VALIDATE_FAIL"


@dataclass
class SupervisorConfig:
    """Tunable parameters for the supervisor FSM."""

    validate_timeout_s: float = 60.0
    # 0.97 rad/s gives a single revolution in ~6.48 s. The ROS wrapper may
    # override at runtime per (operator default 0.3 rad/s on cautious tier).
    announce_spin_w_rad_s: float = 0.97
    announce_spin_duration_s: float = 6.5
    # Camera topic is /oak/rgb/image_raw on this robot's depthai-snap stack;
    # the legacy /camera/color/image_raw remap path was removed in .
    required_topics_alive: tuple[str, ...] = (
        "/scan",
        "/joint_states",
        "/oak/rgb/image_raw",
    )
    arbiter_intent_topic: str = "/par/intents"


@dataclass
class TopicHealth:
    """Snapshot of a single required topic's publisher count at tick time."""

    name: str
    alive: bool
    publisher_count: int = 0


@dataclass
class Decision:
    """Side effects the wrapper must enact after a tick."""

    new_state: SupervisorState | None  # None = stay in current state
    publish_intent: tuple[float, float] | None  # (linear_x, angular_z) or None
    publish_active_mode: str | None  # "IDLE" on entry into IDLE / VALIDATE_FAIL


class SupervisorFSM:
    """Tick-driven supervisor state machine.

    The wrapper feeds the latest sensor freshness on every tick. The FSM
    returns a Decision describing what the wrapper should publish.
    """

    def __init__(self, cfg: SupervisorConfig | None = None) -> None:
        self.cfg = cfg or SupervisorConfig()
        self.state = SupervisorState.BOOT
        self._announce_started_at: float | None = None
        self._idle_mode_published: bool = False

    def _all_required_alive(self, healths: list[TopicHealth]) -> bool:
        required = set(self.cfg.required_topics_alive)
        seen_alive = {h.name for h in healths if h.alive and h.publisher_count >= 1}
        return required.issubset(seen_alive)

    def tick(
        self,
        now: float,
        topic_healths: list[TopicHealth],
        boot_t: float,
    ) -> Decision:
        """Advance the FSM by one tick; return the resulting Decision."""
        # VALIDATE_FAIL is terminal: the operator must restart par-a3-runtime.
        if self.state == SupervisorState.VALIDATE_FAIL:
            return Decision(
                new_state=None,
                publish_intent=None,
                publish_active_mode=None,
            )

        # BOOT auto-transitions to SELF_VALIDATE on the first tick.
        # IMPORTANT (per-advisor ): publish /par/active_mode = "IDLE"
        # IMMEDIATELY at SELF_VALIDATE entry, NOT at the post-spin IDLE entry
        # below. Reason: behaviour nodes default-self-activate when no
        # /par/active_mode is latched (per par_core.ModeState semantics).
        # Publishing IDLE first silences ND, gesture, etc. for the entire
        # 21-second 360 announce window so the supervisor's priority-60 spin
        # intent is not preempted by ND's priority-70 cruise intent.
        if self.state == SupervisorState.BOOT:
            self.state = SupervisorState.SELF_VALIDATE
            return Decision(
                new_state=SupervisorState.SELF_VALIDATE,
                publish_intent=None,
                publish_active_mode="IDLE",
            )

        # SELF_VALIDATE: pass on all-alive, fail on timeout.
        if self.state == SupervisorState.SELF_VALIDATE:
            if self._all_required_alive(topic_healths):
                self.state = SupervisorState.READY_ANNOUNCE
                self._announce_started_at = now
                return Decision(
                    new_state=SupervisorState.READY_ANNOUNCE,
                    publish_intent=(0.0, self.cfg.announce_spin_w_rad_s),
                    publish_active_mode=None,
                )
            if (now - boot_t) >= self.cfg.validate_timeout_s:
                self.state = SupervisorState.VALIDATE_FAIL
                return Decision(
                    new_state=SupervisorState.VALIDATE_FAIL,
                    publish_intent=None,
                    publish_active_mode=None,
                )
            return Decision(
                new_state=None,
                publish_intent=None,
                publish_active_mode=None,
            )

        # READY_ANNOUNCE: republish spin intent until duration elapsed,
        # then drop into IDLE and latch /par/active_mode = "IDLE" once.
        if self.state == SupervisorState.READY_ANNOUNCE:
            assert self._announce_started_at is not None
            if (now - self._announce_started_at) >= self.cfg.announce_spin_duration_s:
                self.state = SupervisorState.IDLE
                # IDLE is already latched from BOOT->SELF_VALIDATE; do NOT
                # re-publish on every IDLE entry to avoid stomping any
                # operator-driven /par/active_mode publish from scene.sh that
                # may have arrived first. The supervisor publishes IDLE once,
                # then never again.
                self._idle_mode_published = True
                return Decision(
                    new_state=SupervisorState.IDLE,
                    publish_intent=None,
                    publish_active_mode=None,
                )
            return Decision(
                new_state=None,
                publish_intent=(0.0, self.cfg.announce_spin_w_rad_s),
                publish_active_mode=None,
            )

        # IDLE is terminal for the supervisor's purposes. The active_mode is
        # latched (TRANSIENT_LOCAL) so a single publish on entry persists for
        # any late-joining subscriber. Operator-driven A/B transitions are
        # handled outside the supervisor by scripts/scene.sh.
        return Decision(
            new_state=None,
            publish_intent=None,
            publish_active_mode=None,
        )
