from par_arbiter.arbiter_core import ArbiterConfig, ScoredIntent, resolve


def mk(age: float, priority: int, v: float = 0.2, source: str = "qr",
 label: str = "GO", confidence: float = 1.0) -> ScoredIntent:
 return ScoredIntent(
 age_s=age, priority=priority, confidence=confidence,
 linear_x=v, angular_z=0.0, source=source, label=label,
 )


def test_empty_returns_none -> None:
 assert resolve([], ArbiterConfig) is None


def test_highest_priority_wins -> None:
 reactive = mk(0.0, 70, v=0.0, source="reactive", label="AVOID")
 qr = mk(0.0, 60, v=0.2, source="qr", label="GO")
 out = resolve([qr, reactive], ArbiterConfig)
 assert out is not None
 _, _, src, label, _ = out
 assert src == "reactive"
 assert label == "AVOID"


def test_stale_intent_is_dropped -> None:
 stale = mk(1.0, 100, source="voice", label="STOP")
 fresh = mk(0.0, 50, source="gesture", label="GO")
 out = resolve([stale, fresh], ArbiterConfig(stale_after_s=0.5))
 assert out is not None
 assert out[2] == "gesture"


def test_decay_lets_fresh_low_priority_win -> None:
 # Old high-priority vs very fresh low-priority. With decay=20 pts/s. # grace=0.1 s, a 0.3s-old priority-80 intent has effective 80 - 20*(0.3-0.1) = 76
 # vs a fresh priority-70 with effective 70. 76 > 70 so old still wins.
 old_high = mk(0.3, 80, source="qr", label="STOP")
 fresh_low = mk(0.0, 70, source="reactive", label="AVOID")
 out = resolve([old_high, fresh_low], ArbiterConfig(decay_rate=20.0, grace_s=0.1))
 assert out is not None
 assert out[2] == "qr"

 # But push the old one older; decay catches up.
 old_high2 = mk(0.8, 80, source="qr", label="STOP")
 fresh_low2 = mk(0.0, 70, source="reactive", label="AVOID")
 out2 = resolve([old_high2, fresh_low2],
 ArbiterConfig(decay_rate=20.0, grace_s=0.1, stale_after_s=2.0))
 assert out2 is not None
 assert out2[2] == "reactive"


def test_tie_break_on_age_then_confidence -> None:
 a = mk(0.1, 60, source="A", confidence=0.5)
 b = mk(0.0, 60, source="B", confidence=0.5) # fresher
 c = mk(0.1, 60, source="C", confidence=0.9)
 out = resolve([a, b, c], ArbiterConfig)
 assert out is not None
 assert out[2] == "B"


def test_operator_stop_at_85_beats_reactive_at_70 -> None:
 """The new STOP priority (85) must win against reactive avoidance (70).

 This is the demo-safety contract surfaced in : the operator must be
 able to halt the robot even while reactive nav is actively manoeuvring.
 """
 reactive = mk(0.0, 70, v=0.2, source="reactive", label="AVOID")
 stop = mk(0.0, 85, v=0.0, source="qr", label="STOP")
 out = resolve([reactive, stop], ArbiterConfig)
 assert out is not None
 v, _, src, label, _ = out
 assert src == "qr" and label == "STOP"
 assert v == 0.0


def test_qr_stop_beats_supervisor_announce -> None:
 """QR-STOP (priority 85) preempts supervisor 360 announce (priority 60).

 Coverage for the 2-mode pivot: the supervisor wrapper publishes an
 ANNOUNCE_360 spin intent at priority 60 with angular_z>0. The operator
 must be able to halt the spin instantly by raising a QR STOP card.
 Same-tick arrival means both are age 0; resolver picks higher priority,
 arbiter output velocity must be zero spin.
 """
 supervisor = ScoredIntent(
 age_s=0.0, priority=60, confidence=1.0,
 linear_x=0.0, angular_z=0.3,
 source="supervisor", label="ANNOUNCE_360",
 )
 qr_stop = ScoredIntent(
 age_s=0.0, priority=85, confidence=1.0,
 linear_x=0.0, angular_z=0.0,
 source="qr", label="STOP",
 )
 out = resolve([supervisor, qr_stop], ArbiterConfig)
 assert out is not None
 v, w, src, label, _ = out
 assert src == "qr"
 assert label == "STOP"
 assert w == 0.0
 assert v == 0.0
