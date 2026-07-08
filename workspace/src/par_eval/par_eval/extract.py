"""Trial data extraction — read a session rosbag + manifest, emit CSVs.

Usage from inside the dev container::

    python -m par_eval.extract \
        --bag    workspace/logs/session_20260512_1430/recording.mcap \
        --manifest workspace/logs/session_20260512_1430/manifest.yaml \
        --out    workspace/logs/session_20260512_1430

The pure-function metric computers below operate on plain dictionaries so they
are unit-testable without ROS bindings. The CLI wrapper at the bottom of the
module lazy-imports ``rosbag2_py`` (only available inside the dev container)
and converts each bag entry to a dictionary before handing it to the
computers.

Manifest schema
---------------
.. code-block:: yaml

 session: -1430
    trials:
      - id: A_0deg_03m_01
        project: A                 # A | B | C | D
        scenario: "0deg, 0.3m, bright"
        expected: GO               # for A: the QR verb expected; for B: the colour;
                                   #   for C: ignored (collision/recovery scoring);
                                   #   for D: the gesture verb
        start_s: 12.5              # bag time when this trial begins
        end_s:   27.0              # bag time when this trial ends

CSV outputs
-----------
``qr_detection.csv``        — trial_id, scenario, runs, detected, accuracy_pct
``traffic_light.csv``       — trial_id, scenario, expected, tp, fp, fn, mean_latency_ms
``reactive.csv``            — trial_id, scenario, duration_s, collisions,
                               collisions_per_min
``cross_modality.csv``      — modality, runs, accuracy_pct, mean_latency_ms

See ```` for how this output complements the
session text log produced by ``par_eval.session_logger``.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Iterable

# ---------------------------------------------------------------------------
# Pure data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Trial:
    """One row of the manifest. ``expected`` interpretation depends on
    ``project``; see module docstring."""

    id: str
    project: str
    scenario: str
    expected: str | None
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_s - self.start_s)


@dataclass
class TrialEvents:
    """All events relevant to one trial, indexed by topic. Each list element
    is ``(t_s, payload_dict)``. ``t_s`` is bag-time, monotonic-aligned."""

    detections: list[tuple[float, dict]] = field(default_factory=list)
    intents: list[tuple[float, dict]] = field(default_factory=list)
    events: list[tuple[float, dict]] = field(default_factory=list)
    signals: list[tuple[float, dict]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Pure-function metric computers
# ---------------------------------------------------------------------------


def slice_events_for_trial(
    raw_events: dict[str, list[tuple[float, dict]]],
    trial: Trial,
) -> TrialEvents:
    """Window all events to ``[trial.start_s, trial.end_s]`` and split by topic.

    ``raw_events`` is a dict keyed by short topic name (``detections``,
    ``intents``, ``events``, ``signals``) → list of (t_s, payload).
    """
    out = TrialEvents()
    for short, target in (
        ("detections", out.detections),
        ("intents", out.intents),
        ("events", out.events),
        ("signals", out.signals),
    ):
        for t_s, payload in raw_events.get(short, []):
            if trial.start_s <= t_s <= trial.end_s:
                target.append((t_s, payload))
    return out


def compute_qr_metrics(trials: Iterable[Trial], windowed: dict[str, TrialEvents]) -> list[dict]:
    """For each Project-A trial, count the number of QR detections matching
    the expected verb and compute accuracy as detections / runs.

    A "run" inside one trial is one operator card-show. We approximate run
    count as the number of distinct fresh detection bursts (each burst = one
    or more events for the same payload within ~1.0 s). This is a weaker
    metric than a pre-counted ground truth but works without per-card
    timestamps.

    Returns one row per trial.
    """
    rows: list[dict] = []
    for trial in trials:
        if trial.project != "A":
            continue
        events = windowed.get(trial.id, TrialEvents()).detections
        runs, detected = _count_runs_and_matches(events, trial.expected, gap_s=1.0)
        accuracy = (100.0 * detected / runs) if runs else 0.0
        rows.append(
            {
                "trial_id": trial.id,
                "scenario": trial.scenario,
                "runs": runs,
                "detected": detected,
                "accuracy_pct": round(accuracy, 1),
            }
        )
    return rows


def _count_runs_and_matches(
    events: list[tuple[float, dict]],
    expected: str | None,
    gap_s: float,
) -> tuple[int, int]:
    """Split events into temporal bursts (gap_s apart) and count: total
    bursts, and how many bursts contain at least one matching payload."""
    if not events:
        return 0, 0
    runs = 0
    detected = 0
    last_t: float | None = None
    burst_matched = False
    for t, payload in events:
        if last_t is None or (t - last_t) > gap_s:
            if last_t is not None:
                runs += 1
                if burst_matched:
                    detected += 1
            burst_matched = False
        if expected is not None and payload.get("payload") == expected:
            burst_matched = True
        last_t = t
    runs += 1
    if burst_matched:
        detected += 1
    return runs, detected


def compute_traffic_metrics(
    trials: Iterable[Trial], windowed: dict[str, TrialEvents]
) -> list[dict]:
    """For each Project-B trial, classify each SignalState transition as
    TP / FP / FN against the expected colour, and measure response latency
    from SignalState to the next CommandIntent on the traffic source."""
    rows: list[dict] = []
    for trial in trials:
        if trial.project != "B":
            continue
        ev = windowed.get(trial.id, TrialEvents())
        tp, fp, fn = _classify_signal_states(ev.signals, trial.expected)
        latency_ms = _mean_signal_to_intent_latency_ms(ev.signals, ev.intents)
        rows.append(
            {
                "trial_id": trial.id,
                "scenario": trial.scenario,
                "expected": trial.expected or "",
                "tp": tp,
                "fp": fp,
                "fn": fn,
                "mean_latency_ms": round(latency_ms, 1) if latency_ms is not None else "",
            }
        )
    return rows


def _classify_signal_states(
    signals: list[tuple[float, dict]],
    expected: str | None,
) -> tuple[int, int, int]:
    """Count TP / FP / FN across a list of SignalState observations.

    A TP is a state observation that matches the expected colour.
    A FP is a non-UNKNOWN state observation that does not match.
    A FN is bracketed by absence: per trial, FN = 1 if no observation
    matched the expected colour, else 0.
    """
    if expected is None:
        return 0, 0, 0
    tp = 0
    fp = 0
    matched = False
    for _t, payload in signals:
        state = payload.get("state", "UNKNOWN")
        if state == expected:
            tp += 1
            matched = True
        elif state != "UNKNOWN":
            fp += 1
    fn = 0 if matched else 1
    return tp, fp, fn


def _mean_signal_to_intent_latency_ms(
    signals: list[tuple[float, dict]],
    intents: list[tuple[float, dict]],
) -> float | None:
    """For each signal observation, find the next traffic-source intent
    timestamp and average the deltas. Returns None when nothing pairs up."""
    if not signals or not intents:
        return None
    deltas: list[float] = []
    for sig_t, _ in signals:
        cand = next(
            ((it_t, it) for it_t, it in intents if it_t >= sig_t and it.get("source") == "traffic"),
            None,
        )
        if cand is not None:
            deltas.append((cand[0] - sig_t) * 1000.0)
    if not deltas:
        return None
    return sum(deltas) / len(deltas)


def compute_reactive_metrics(
    trials: Iterable[Trial], windowed: dict[str, TrialEvents]
) -> list[dict]:
    """For each Project-C trial, count collision-class events and compute
    collisions-per-minute.

    Collision-class events are TrialEvent rows whose ``event`` field starts
    with ``collision`` or whose detail mentions ``tof``/``lidar_stop``.
    """
    rows: list[dict] = []
    for trial in trials:
        if trial.project != "C":
            continue
        ev = windowed.get(trial.id, TrialEvents()).events
        collisions = sum(1 for _t, p in ev if _is_collision_event(p))
        cpm = collisions / (trial.duration_s / 60.0) if trial.duration_s > 0 else 0.0
        rows.append(
            {
                "trial_id": trial.id,
                "scenario": trial.scenario,
                "duration_s": round(trial.duration_s, 1),
                "collisions": collisions,
                "collisions_per_min": round(cpm, 2),
            }
        )
    return rows


def _is_collision_event(payload: dict) -> bool:
    if payload.get("event", "").lower().startswith("collision"):
        return True
    detail = payload.get("detail", "").lower()
    return "tof" in detail or "lidar_stop" in detail


def compute_cross_modality(
    trials: Iterable[Trial], windowed: dict[str, TrialEvents]
) -> list[dict]:
    """Aggregate accuracy + latency by modality for the cross-modality
    ablation. Trials whose ``project`` is ``A`` count as the QR modality;
    project ``D`` counts as Hand-gesture."""
    by_mod: dict[str, dict] = defaultdict(lambda: {"runs": 0, "detected": 0, "latencies_ms": []})
    for trial in trials:
        if trial.project not in ("A", "D"):
            continue
        modality = "QR" if trial.project == "A" else "Hand"
        ev = windowed.get(trial.id, TrialEvents())
        runs, detected = _count_runs_and_matches(ev.detections, trial.expected, gap_s=1.0)
        by_mod[modality]["runs"] += runs
        by_mod[modality]["detected"] += detected
        latency = _mean_detection_to_intent_latency_ms(ev.detections, ev.intents, modality)
        if latency is not None:
            by_mod[modality]["latencies_ms"].append(latency)

    rows = []
    for modality, agg in by_mod.items():
        runs = agg["runs"]
        accuracy = (100.0 * agg["detected"] / runs) if runs else 0.0
        latencies = agg["latencies_ms"]
        mean_latency = sum(latencies) / len(latencies) if latencies else None
        rows.append(
            {
                "modality": modality,
                "runs": runs,
                "accuracy_pct": round(accuracy, 1),
                "mean_latency_ms": round(mean_latency, 1) if mean_latency is not None else "",
            }
        )
    return rows


def _mean_detection_to_intent_latency_ms(
    detections: list[tuple[float, dict]],
    intents: list[tuple[float, dict]],
    modality: str,
) -> float | None:
    expected_source = "qr" if modality == "QR" else "gesture"
    if not detections or not intents:
        return None
    deltas: list[float] = []
    for det_t, _ in detections:
        cand = next(
            (
                it_t
                for it_t, it in intents
                if it_t >= det_t and it.get("source") == expected_source
            ),
            None,
        )
        if cand is not None:
            deltas.append((cand - det_t) * 1000.0)
    if not deltas:
        return None
    return sum(deltas) / len(deltas)


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------


def write_csv(path: str, rows: list[dict], columns: list[str]) -> None:
    if not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# Manifest loader
# ---------------------------------------------------------------------------


def load_manifest(path: str) -> tuple[str, list[Trial]]:
    """Read the YAML manifest and return (session_id, trials). YAML is
    lazy-imported because ``yaml`` is not in par_eval's package.xml; the
    operator running the CLI has ``pyyaml`` available in the dev container.
    """
    import yaml  # type: ignore[import-not-found]  # noqa: PLC0415

    with open(path) as fh:
        data = yaml.safe_load(fh)
    session = str(data.get("session", "unknown"))
    trials = [Trial(**t) for t in data.get("trials", [])]
    return session, trials


# ---------------------------------------------------------------------------
# Bag → dict bridge (lazy-imported rosbag2_py)
# ---------------------------------------------------------------------------


def _read_bag(bag_path: str) -> dict[str, list[tuple[float, dict]]]:
    """Read an mcap or sqlite3 bag and return a per-topic list of
    (t_s, payload_dict). Lazy-imports rosbag2_py and rclpy_serialization.

    The function deliberately knows only about the four topic short-names
    used elsewhere in the module so a caller without ROS installed can run
    the pure-function tests via ``compute_*`` without touching this code.
    """
    from rclpy.serialization import deserialize_message  # type: ignore[import-not-found]
    from rosbag2_py import (  # type: ignore[import-not-found]
        ConverterOptions,
        SequentialReader,
        StorageOptions,
    )
    from rosidl_runtime_py.utilities import get_message  # type: ignore[import-not-found]

    storage = StorageOptions(uri=bag_path)
    converter = ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage, converter)

    type_map = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    short_for = {
        "/par/detections": "detections",
        "/par/intents": "intents",
        "/par/events": "events",
        "/par/signal_state": "signals",
    }

    out: dict[str, list[tuple[float, dict]]] = defaultdict(list)
    bag_t0: float | None = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        short = short_for.get(topic)
        if short is None:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_type)
        t_s = t_ns / 1e9
        if bag_t0 is None:
            bag_t0 = t_s
        rel = t_s - bag_t0
        out[short].append((rel, _msg_to_dict(msg)))
    return dict(out)


def _msg_to_dict(msg) -> dict:
    """Pluck the fields each computer cares about. Adding a field here is
    cheap; the dict shape is what the pure functions consume."""
    payload: dict = {}
    for name in ("source", "payload", "label", "priority", "confidence",
                 "state", "event", "detail", "phase"):
        if hasattr(msg, name):
            payload[name] = getattr(msg, name)
    if hasattr(msg, "cmd"):
        cmd = msg.cmd
        if hasattr(cmd, "linear"):
            payload["linear_x"] = cmd.linear.x
            payload["angular_z"] = cmd.angular.z
    return payload


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("--bag", required=True, help="Path to the recording (mcap or sqlite3 dir)")
    parser.add_argument("--manifest", required=True, help="Path to the trial manifest YAML")
    parser.add_argument("--out", required=True, help="Output directory for CSVs")
    ns = parser.parse_args(args)

    session, trials = load_manifest(ns.manifest)
    raw = _read_bag(ns.bag)

    windowed = {trial.id: slice_events_for_trial(raw, trial) for trial in trials}

    write_csv(
        os.path.join(ns.out, "qr_detection.csv"),
        compute_qr_metrics(trials, windowed),
        ["trial_id", "scenario", "runs", "detected", "accuracy_pct"],
    )
    write_csv(
        os.path.join(ns.out, "traffic_light.csv"),
        compute_traffic_metrics(trials, windowed),
        ["trial_id", "scenario", "expected", "tp", "fp", "fn", "mean_latency_ms"],
    )
    write_csv(
        os.path.join(ns.out, "reactive.csv"),
        compute_reactive_metrics(trials, windowed),
        ["trial_id", "scenario", "duration_s", "collisions", "collisions_per_min"],
    )
    write_csv(
        os.path.join(ns.out, "cross_modality.csv"),
        compute_cross_modality(trials, windowed),
        ["modality", "runs", "accuracy_pct", "mean_latency_ms"],
    )

    summary = {
        "session": session,
        "trial_count": len(trials),
        "outputs": ["qr_detection.csv", "traffic_light.csv",
                    "reactive.csv", "cross_modality.csv"],
    }
    with open(os.path.join(ns.out, "extract_summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(f"[par_eval.extract] wrote 4 CSVs to {ns.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
