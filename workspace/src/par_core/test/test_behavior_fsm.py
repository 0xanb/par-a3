from par_core import BehaviorFSM


def test_transition_fires_when_guard_true -> None:
 fsm = BehaviorFSM(state="DRIVING")
 fsm.add("DRIVING", "STOPPED", guard=lambda: True)
 assert fsm.tick == "STOPPED"


def test_transition_blocks_when_guard_false -> None:
 fsm = BehaviorFSM(state="DRIVING")
 fsm.add("DRIVING", "STOPPED", guard=lambda: False)
 assert fsm.tick == "DRIVING"


def test_on_enter_on_exit_fire_in_order -> None:
 events: list[str] = []
 fsm = BehaviorFSM(state="A")
 fsm.on_exit = {"A": lambda: events.append("exit-A")}
 fsm.on_enter = {"B": lambda: events.append("enter-B")}
 fsm.add("A", "B", guard=lambda: True)
 fsm.tick
 assert events == ["exit-A", "enter-B"]
