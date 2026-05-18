"""Host-runnable unit tests for supervisor_core (post pivot).

Covers every transition in the simplified FSM:
 BOOT -> SELF_VALIDATE -> READY_ANNOUNCE -> IDLE
 SELF_VALIDATE -> VALIDATE_FAIL on timeout

The /buttons subscriber, /leds publisher, and MODE_A / MODE_B states from
the sprint were removed because the rosbot_ros snap on this robot
exposes neither /buttons nor /ledsmd ). Mode switching
is now driven externally by scripts/scene.sh; the supervisor only owns the
cold-boot sequence.

No ROS imports.
"""
from __future__ import annotations

from par_supervisor.supervisor_core import (
 SupervisorConfig,
 SupervisorFSM,
 SupervisorState,
 TopicHealth,
)


# --- Helpers ---------------------------------------------------------------

def healthy_topics(cfg: SupervisorConfig) -> list[TopicHealth]:
 """One alive TopicHealth per required topic."""
 return [TopicHealth(name=t, alive=True, publisher_count=1) for t in cfg.required_topics_alive]


def unhealthy_topics(cfg: SupervisorConfig) -> list[TopicHealth]:
 """All required topics absent (no publishers)."""
 return [TopicHealth(name=t, alive=False, publisher_count=0) for t in cfg.required_topics_alive]


def fsm_in_state(state: SupervisorState, cfg: SupervisorConfig | None = None) -> SupervisorFSM:
 """Build an FSM and force it into the given state by walking the boot path."""
 fsm = SupervisorFSM(cfg)
 cfg = fsm.cfg
 if state == SupervisorState.BOOT:
 return fsm
 # BOOT -> SELF_VALIDATE
 fsm.tick(now=0.0, topic_healths=healthy_topics(cfg), boot_t=0.0)
 if state == SupervisorState.SELF_VALIDATE:
 return fsm
 # SELF_VALIDATE -> READY_ANNOUNCE (announce starts at now=0.1)
 fsm.tick(now=0.1, topic_healths=healthy_topics(cfg), boot_t=0.0)
 if state == SupervisorState.READY_ANNOUNCE:
 return fsm
 # READY_ANNOUNCE -> IDLE after spin duration
 fsm.tick(
 now=0.1 + cfg.announce_spin_duration_s + 0.01,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 if state == SupervisorState.IDLE:
 return fsm
 raise AssertionError(f"unsupported state for fixture: {state}")


# --- BOOT and SELF_VALIDATE -----------------------------------------------

def test_boot_transitions_to_self_validate_on_first_tick -> None:
 fsm = SupervisorFSM
 decision = fsm.tick(
 now=0.0,
 topic_healths=[],
 boot_t=0.0,
 )
 assert decision.new_state == SupervisorState.SELF_VALIDATE
 assert fsm.state == SupervisorState.SELF_VALIDATE
 # Publish /par/active_mode = "IDLE" IMMEDIATELY on SELF_VALIDATE entry
 # so behaviour-node mode gates close before the supervisor's 360 announce
 # starts. Reason: par_core.ModeState defaults current_mode to its own
 # active_in_mode if no /par/active_mode has been latched, which would
 # let ND emit priority-70 cruise intents during the supervisor's
 # priority-60 spin (per-advisor).
 assert decision.publish_active_mode == "IDLE"


def test_self_validate_passes_when_all_topics_alive -> None:
 cfg = SupervisorConfig
 fsm = SupervisorFSM(cfg)
 # First tick: BOOT -> SELF_VALIDATE.
 fsm.tick(now=0.0, topic_healths=[], boot_t=0.0)
 # Second tick: all required topics alive -> READY_ANNOUNCE with spin intent.
 decision = fsm.tick(
 now=0.5,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 assert decision.new_state == SupervisorState.READY_ANNOUNCE
 assert fsm.state == SupervisorState.READY_ANNOUNCE


def test_self_validate_times_out_to_validate_fail -> None:
 cfg = SupervisorConfig(validate_timeout_s=5.0)
 fsm = SupervisorFSM(cfg)
 fsm.tick(now=0.0, topic_healths=[], boot_t=0.0)
 # Still missing topics after the timeout window.
 decision = fsm.tick(
 now=5.5,
 topic_healths=unhealthy_topics(cfg),
 boot_t=0.0,
 )
 assert decision.new_state == SupervisorState.VALIDATE_FAIL
 assert fsm.state == SupervisorState.VALIDATE_FAIL
 # No active_mode publish on validate-fail (terminal state, supervisor stuck).
 assert decision.publish_active_mode is None


# --- READY_ANNOUNCE --------------------------------------------------------

def test_ready_announce_publishes_spin_intent -> None:
 cfg = SupervisorConfig(announce_spin_w_rad_s=0.97, announce_spin_duration_s=6.5)
 fsm = SupervisorFSM(cfg)
 fsm.tick(now=0.0, topic_healths=[], boot_t=0.0)
 transition = fsm.tick(
 now=0.1,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 assert transition.new_state == SupervisorState.READY_ANNOUNCE
 assert transition.publish_intent is not None
 linear, angular = transition.publish_intent
 assert linear == 0.0
 assert angular == cfg.announce_spin_w_rad_s
 # Subsequent ticks during the spin re-publish the intent at the same rate.
 holding = fsm.tick(
 now=2.0,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 assert holding.publish_intent == (0.0, cfg.announce_spin_w_rad_s)
 assert holding.new_state is None


def test_ready_announce_to_idle_after_duration -> None:
 cfg = SupervisorConfig(announce_spin_duration_s=6.5)
 fsm = SupervisorFSM(cfg)
 fsm.tick(now=0.0, topic_healths=[], boot_t=0.0)
 fsm.tick(now=0.1, topic_healths=healthy_topics(cfg), boot_t=0.0)
 decision = fsm.tick(
 now=0.1 + cfg.announce_spin_duration_s + 0.05,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 assert decision.new_state == SupervisorState.IDLE
 # IDLE is NOT re-published on entry; it was already latched at
 # SELF_VALIDATE entry. Re-publishing here could stomp an operator-driven
 # /par/active_mode publish from scene.sh that may have arrived first.
 assert decision.publish_active_mode is None
 assert decision.publish_intent is None


# --- IDLE latch behaviour --------------------------------------------------

def test_idle_published_at_self_validate_entry_only -> None:
 """publish_active_mode='IDLE' fires exactly once: at BOOT->SELF_VALIDATE.

 Subsequent transitions (SELF_VALIDATE->READY_ANNOUNCE, READY_ANNOUNCE->IDLE)
 must NOT re-publish IDLE; the latched TRANSIENT_LOCAL message from the
 first publish persists for any subscriber, and re-publishing risks
 stomping an operator-driven /par/active_mode publish from scene.sh that
 may have arrived in between (e.g. operator pre-stages mode A while
 supervisor is still spinning). Single-source-of-truth via single publish.
 """
 cfg = SupervisorConfig(announce_spin_duration_s=6.5)
 fsm = SupervisorFSM(cfg)
 boot = fsm.tick(now=0.0, topic_healths=[], boot_t=0.0)
 assert boot.publish_active_mode == "IDLE" # the one and only publish
 validate = fsm.tick(now=0.1, topic_healths=healthy_topics(cfg), boot_t=0.0)
 assert validate.publish_active_mode is None # entry into READY_ANNOUNCE
 finish = fsm.tick(
 now=0.1 + cfg.announce_spin_duration_s + 0.05,
 topic_healths=healthy_topics(cfg),
 boot_t=0.0,
 )
 assert finish.publish_active_mode is None # entry into IDLE


def test_idle_does_not_re_publish_active_mode_on_subsequent_ticks -> None:
 """Once in IDLE, all subsequent ticks return a no-op Decision.

 The /par/active_mode topic is TRANSIENT_LOCAL so a single publish is
 persistent for any late-joining subscriber. Re-publishing on every tick
 would spam the latched-QoS history pointlessly.
 """
 fsm = fsm_in_state(SupervisorState.IDLE)
 # Several ticks well after the IDLE transition; nothing should publish.
 for t in (20.0, 30.0, 60.0):
 decision = fsm.tick(
 now=t,
 topic_healths=healthy_topics(fsm.cfg),
 boot_t=0.0,
 )
 assert decision.new_state is None
 assert decision.publish_active_mode is None
 assert decision.publish_intent is None
 assert fsm.state == SupervisorState.IDLE
