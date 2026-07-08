"""Priority arbiter core — pure function, no ROS.

Selection rule
--------------
1. Drop intents older than ``stale_after_s``.
2. For each remaining intent, compute an effective priority:
       effective = intent.priority - decay_rate * max(0, age - grace)
   where ``grace`` is a short window during which an intent does not decay.
3. Break ties by freshness, then by confidence.
4. Winning intent's ``cmd`` is the arbiter's output. If no intent is eligible,
   return zero velocity.

Why freshness decay
-------------------
Without decay, a QR-stamped STOP could dominate forever. With decay, a TURN
that fires at t=0 yields to a REACTIVE avoid at t=0.4 if the avoid has higher
priority and the original turn is now 0.4 s old. This is exactly the behaviour
we want on hardware.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ScoredIntent:
    """A candidate intent with its age. Age is what the arbiter uses to decay."""
    age_s: float
    priority: int
    confidence: float
    linear_x: float
    angular_z: float
    source: str
    label: str


@dataclass
class ArbiterConfig:
    stale_after_s: float = 0.5
    grace_s: float = 0.1
    decay_rate: float = 20.0   # priority points per second past the grace window


def resolve(candidates: list[ScoredIntent], cfg: ArbiterConfig
            ) -> tuple[float, float, str, str, float] | None:
    """Return (linear_x, angular_z, source, label, effective_priority) or None.

    None -> nothing fresh enough to drive the robot; caller should publish zero.
    """
    if not candidates:
        return None
    eligible: list[tuple[float, ScoredIntent]] = []
    for c in candidates:
        if c.age_s > cfg.stale_after_s:
            continue
        effective = float(c.priority)
        if c.age_s > cfg.grace_s:
            effective -= cfg.decay_rate * (c.age_s - cfg.grace_s)
        eligible.append((effective, c))
    if not eligible:
        return None
    # Highest effective priority wins. Ties: freshest, then most confident.
    eligible.sort(key=lambda p: (-p[0], p[1].age_s, -p[1].confidence))
    eff, winner = eligible[0]
    return winner.linear_x, winner.angular_z, winner.source, winner.label, eff
