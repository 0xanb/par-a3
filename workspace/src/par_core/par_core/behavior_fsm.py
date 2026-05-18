"""Minimal FSM base class used by the project-specific state machines.

Each project node builds its own concrete FSM on top of this. Keeping the
surface area small means the report can show the exact state diagrams without
burying them in framework code.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Transition:
 src: str
 dst: str
 guard: Callable[, bool]


@dataclass
class BehaviorFSM:
 state: str
 transitions: list[Transition] = field(default_factory=list)
 on_enter: dict[str, Callable[[], None]] = field(default_factory=dict)
 on_exit: dict[str, Callable[[], None]] = field(default_factory=dict)

 def add(self, src: str, dst: str, guard: Callable[, bool]) -> None:
 self.transitions.append(Transition(src, dst, guard))

 def tick(self, *args, **kwargs) -> str:
 for t in self.transitions:
 if t.src == self.state and t.guard(*args, **kwargs):
 self._leave(self.state)
 self.state = t.dst
 self._enter(self.state)
 break
 return self.state

 def _enter(self, s: str) -> None:
 cb = self.on_enter.get(s)
 if cb:
 cb

 def _leave(self, s: str) -> None:
 cb = self.on_exit.get(s)
 if cb:
 cb
