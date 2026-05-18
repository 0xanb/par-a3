"""Operator deadman — gamepad / keyboard heartbeat gates the arbiter.

The operator holds a button (gamepad `L1`) or a keyboard key (`space`) while
the robot is meant to be moving. Release -> zero velocity within one tick.

Heartbeat contract
------------------
Publisher: any teleop node
Topic: ``/par/deadman`` std_msgs/Empty >= 10 Hz while held
Consumer: Arbiter / SafetyLayer.armed = (t_now - t_last_heartbeat) < 0.3

On boot, armed is False. This means: **the robot cannot move until an
operator explicitly presses the deadman**. No auto-arm, no "default on"
behaviour, no matter what the behaviour stack publishes.
"""
from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class Deadman:
 timeout_s: float = 0.30
 _last_heartbeat: float | None = None

 def heartbeat(self, now_s: float | None = None) -> None:
 self._last_heartbeat = now_s if now_s is not None else time.monotonic

 def is_armed(self, now_s: float | None = None) -> bool:
 if self._last_heartbeat is None:
 return False
 now = now_s if now_s is not None else time.monotonic
 return (now - self._last_heartbeat) < self.timeout_s
