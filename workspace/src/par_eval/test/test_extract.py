"""Pure-function tests for par_eval.extract.

The bag-reader path (``_read_bag``) lazy-imports rosbag2_py and is exercised
on the robot. Everything below operates on synthetic dictionaries.
"""
from par_eval.extract import (
 Trial,
 TrialEvents,
 compute_cross_modality,
 compute_qr_metrics,
 compute_reactive_metrics,
 compute_traffic_metrics,
 slice_events_for_trial,
)


def make_trial(**overrides) -> Trial:
 base = dict(
 id="A_test_01",
 project="A",
 scenario="0deg, 0.6m, bright",
 expected="GO",
 start_s=0.0,
 end_s=10.0,
 )
 base.update(overrides)
 return Trial(**base)


# ---------------------------------------------------------------------------
# Windowing
# ---------------------------------------------------------------------------


def test_slice_keeps_only_in_window_events -> None:
 raw = {
 "detections": [(0.5, {"payload": "GO"}), (12.0, {"payload": "STOP"})],
 }
 out = slice_events_for_trial(raw, make_trial(start_s=0.0, end_s=10.0))
 assert len(out.detections) == 1
 assert out.detections[0][1]["payload"] == "GO"


def test_slice_handles_missing_topics -> None:
 out = slice_events_for_trial({}, make_trial)
 assert out.detections == []
 assert out.intents == []


# ---------------------------------------------------------------------------
# QR detection accuracy
# ---------------------------------------------------------------------------


def test_qr_metrics_one_match_one_run -> None:
 trials = [make_trial(expected="GO")]
 windowed = {
 "A_test_01": TrialEvents(detections=[(1.0, {"payload": "GO"})]),
 }
 rows = compute_qr_metrics(trials, windowed)
 assert rows == [
 {"trial_id": "A_test_01", "scenario": "0deg, 0.6m, bright",
 "runs": 1, "detected": 1, "accuracy_pct": 100.0}
 ]


def test_qr_metrics_separates_bursts_by_gap -> None:
 """Two card-shows separated by >1.0 s = two runs."""
 trials = [make_trial(expected="GO")]
 windowed = {
 "A_test_01": TrialEvents(
 detections=[
 (1.0, {"payload": "GO"}), # run 1
 (1.2, {"payload": "GO"}), # same burst
 (5.0, {"payload": "STOP"}), # run 2 (gap > 1.0 s; payload mismatch)
 ]
 ),
 }
 rows = compute_qr_metrics(trials, windowed)
 assert rows[0]["runs"] == 2
 assert rows[0]["detected"] == 1
 assert rows[0]["accuracy_pct"] == 50.0


def test_qr_metrics_no_events_returns_zero_runs -> None:
 trials = [make_trial(expected="GO")]
 windowed = {"A_test_01": TrialEvents}
 rows = compute_qr_metrics(trials, windowed)
 assert rows[0]["runs"] == 0
 assert rows[0]["detected"] == 0


def test_qr_metrics_skips_non_a_projects -> None:
 trials = [make_trial(id="B_x", project="B", expected="RED")]
 rows = compute_qr_metrics(trials, {})
 assert rows == []


# ---------------------------------------------------------------------------
# Traffic-light TP/FP/FN
# ---------------------------------------------------------------------------


def test_traffic_metrics_tp_fp_fn -> None:
 trials = [make_trial(id="B_01", project="B", expected="RED",
 scenario="bright", start_s=0.0, end_s=10.0)]
 windowed = {
 "B_01": TrialEvents(
 signals=[
 (1.0, {"state": "RED"}), # TP
 (2.0, {"state": "RED"}), # TP again — counts every observation
 (3.0, {"state": "GREEN"}), # FP
 (4.0, {"state": "UNKNOWN"}), # ignored
 ],
 ),
 }
 rows = compute_traffic_metrics(trials, windowed)
 assert rows[0]["tp"] == 2
 assert rows[0]["fp"] == 1
 assert rows[0]["fn"] == 0


def test_traffic_metrics_fn_when_no_match -> None:
 trials = [make_trial(id="B_02", project="B", expected="GREEN")]
 windowed = {
 "B_02": TrialEvents(
 signals=[(1.0, {"state": "RED"}), (2.0, {"state": "RED"})]
 ),
 }
 rows = compute_traffic_metrics(trials, windowed)
 assert rows[0]["tp"] == 0
 assert rows[0]["fp"] == 2
 assert rows[0]["fn"] == 1


def test_traffic_metrics_latency_pairs_signal_with_intent -> None:
 trials = [make_trial(id="B_03", project="B", expected="GREEN",
 start_s=0.0, end_s=10.0)]
 windowed = {
 "B_03": TrialEvents(
 signals=[(1.000, {"state": "GREEN"})],
 intents=[(1.150, {"source": "traffic", "label": "GREEN"})], # 150 ms later
 ),
 }
 rows = compute_traffic_metrics(trials, windowed)
 assert rows[0]["mean_latency_ms"] == 150.0


# ---------------------------------------------------------------------------
# Reactive collisions
# ---------------------------------------------------------------------------


def test_reactive_metrics_counts_collision_events -> None:
 trials = [make_trial(id="C_01", project="C", scenario="cluttered",
 start_s=0.0, end_s=300.0)]
 windowed = {
 "C_01": TrialEvents(
 events=[
 (10.0, {"event": "collision", "detail": "tof"}),
 (20.0, {"event": "stale_perception", "detail": ""}),
 (30.0, {"event": "note", "detail": "lidar_stop reasons logged"}),
 ],
 ),
 }
 rows = compute_reactive_metrics(trials, windowed)
 assert rows[0]["collisions"] == 2 # the tof event + the lidar_stop note
 assert rows[0]["duration_s"] == 300.0
 assert rows[0]["collisions_per_min"] == 0.4


def test_reactive_metrics_zero_duration_safe -> None:
 trials = [make_trial(id="C_zero", project="C", start_s=5.0, end_s=5.0)]
 windowed = {"C_zero": TrialEvents}
 rows = compute_reactive_metrics(trials, windowed)
 assert rows[0]["collisions_per_min"] == 0.0


# ---------------------------------------------------------------------------
# Cross-modality
# ---------------------------------------------------------------------------


def test_cross_modality_aggregates_qr_and_hand -> None:
 trials = [
 make_trial(id="A_01", project="A", expected="STOP"),
 make_trial(id="D_01", project="D", expected="STOP",
 start_s=0.0, end_s=10.0),
 ]
 windowed = {
 "A_01": TrialEvents(
 detections=[(1.0, {"payload": "STOP"})],
 intents=[(1.05, {"source": "qr", "label": "STOP"})],
 ),
 "D_01": TrialEvents(
 detections=[(1.0, {"payload": "STOP"})],
 intents=[(1.20, {"source": "gesture", "label": "STOP"})],
 ),
 }
 rows = compute_cross_modality(trials, windowed)
 by_mod = {r["modality"]: r for r in rows}
 assert by_mod["QR"]["accuracy_pct"] == 100.0
 assert by_mod["Hand"]["accuracy_pct"] == 100.0
 # QR latency is 50 ms, Hand is 200 ms.
 assert by_mod["QR"]["mean_latency_ms"] == 50.0
 assert by_mod["Hand"]["mean_latency_ms"] == 200.0
