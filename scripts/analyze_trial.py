#!/usr/bin/env python3
"""Parse a PAR-A3 session log into Project C trial metrics.

Reads ``<session_dir>/log.txt`` (session_logger output) and an optional
``trial_config.yaml`` (written by ``run_trial.sh``), then emits:

  * ``<session_dir>/metrics.yaml``  — full metric dict (machine-readable)
  * ``<session_dir>/debrief.md``    — operator-fillable template seeded with metrics
  * ``report/data/trials.csv``      — appended one-line CSV row for the campaign

Metrics computed:
  - collisions_per_min          (annotated collisions + 0.5 × long tof_clamp events)
  - recovery_success_rate       (DEAD_END_* paired with subsequent reactive/HSGR or FORWARD within 10 s)
  - dynamic_response_latency_s  (first non-cruise winner after operator's dyn_entered annotation)
  - wedge_dwell_p50, wedge_dwell_max  (median + max DEAD_END → recovery exit)
  - algo_state_distribution     (% of trial in each winner=reactive/<label> bucket)
  - duration_s                  (end_t - start_t in log)

Coverage (m²/min) requires /odometry/filtered which session_logger does not yet
record. The ``coverage_m2_per_min`` field is left null until that subscription
lands; documented in HANDOFF as a Phase 2.5 follow-up.

Usage:
    python3 scripts/analyze_trial.py <session_dir> [--csv report/data/trials.csv]

Pure-function entry points (testable without ROS):
    parse_log_lines(text)                 — text -> list[Row]
    compute_metrics(rows, config)         — list[Row] + config -> dict
    seed_debrief(metrics, config)         — dict -> markdown string
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as _dt
import math
import pathlib
import re
import sys
from collections import Counter
from typing import Any


# --- Parser ----------------------------------------------------------------

@dataclasses.dataclass
class Row:
    """One log.txt line, normalised."""
    t_s: float            # epoch seconds (parsed from "YYYY-MM-DD HH:MM:SS.fff")
    kind: str             # one of: intent, detect, event, mode, log
    source: str           # source / scenario / level / "-"
    name: str             # label / event / mode / log-name
    detail: str           # everything after the name column

    # Convenience helpers
    @property
    def winner(self) -> tuple[str, str] | None:
        """Parse 'winner=src/label' from arbiter_switch / arbiter log rows."""
        m = re.search(r"winner=([^/\s]+)/(\S+)", self.detail)
        if m:
            return m.group(1), m.group(2)
        return None

    @property
    def clamp_reason(self) -> str | None:
        m = re.search(r"clamp=(\S+)", self.detail)
        if m:
            return m.group(1)
        return None

    @property
    def linear_x(self) -> float | None:
        m = re.search(r"v=([-+]?[\d.]+)", self.detail)
        return float(m.group(1)) if m else None

    @property
    def odom_xy(self) -> tuple[float, float] | None:
        """Parse 'x=N y=N' from an odom row's detail field."""
        x = re.search(r"x=([-+]?[\d.]+)", self.detail)
        y = re.search(r"y=([-+]?[\d.]+)", self.detail)
        if x and y:
            return float(x.group(1)), float(y.group(1))
        return None


_LINE_RE = re.compile(
    r"^(?P<ts>\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\.\d+)\s+"
    r"(?P<kind>intent|detect|event|mode|log|odom)\s+"
    r"(?P<source>\S+)\s+"
    r"(?P<name>\S+)\s*"
    r"(?P<rest>.*)$"
)


def _parse_ts(ts: str) -> float:
    return _dt.datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f").timestamp()


def parse_log_lines(text: str) -> list[Row]:
    rows: list[Row] = []
    for raw in text.splitlines():
        m = _LINE_RE.match(raw)
        if not m:
            continue
        try:
            t_s = _parse_ts(m.group("ts"))
        except ValueError:
            continue
        rows.append(Row(
            t_s=t_s,
            kind=m.group("kind"),
            source=m.group("source"),
            name=m.group("name"),
            detail=m.group("rest").strip(),
        ))
    return rows


# --- Metric computation ----------------------------------------------------

def _slice_to_drive_window(rows: list[Row], window_s: float) -> list[Row]:
    """Return only rows within [t_drive_start, t_drive_start + window_s].

    t_drive_start = first arbiter_switch event timestamp (when the arbiter
    first picked a non-default winner — i.e. when drive activity begins).
    Falls back to the first row timestamp if no arbiter_switch is present.

    Used for apples-to-apples comparison across trials whose rosbag durations
    differ due to pre/post tail variance. Default trial drive is 150 s.
    """
    drive_start: float | None = None
    for r in rows:
        if r.kind == "event" and r.name == "arbiter_switch":
            drive_start = r.t_s
            break
    if drive_start is None:
        if not rows:
            return rows
        drive_start = rows[0].t_s
    drive_end = drive_start + window_s
    return [r for r in rows if drive_start <= r.t_s <= drive_end]


def _trial_window(rows: list[Row]) -> tuple[float, float]:
    """Earliest + latest timestamp in the log."""
    if not rows:
        return 0.0, 0.0
    return rows[0].t_s, rows[-1].t_s


def _collision_count(rows: list[Row]) -> int:
    """Operator-annotated TrialEvents with event='collision'."""
    return sum(1 for r in rows if r.kind == "event" and r.name == "collision")


def _anomaly_event_count(rows: list[Row], event_name: str) -> int:
    """Count par_anomaly-emitted TrialEvents by event name.

    Used for the three new ground-truth detection columns:
    ``tilt``, ``collision_impact``, ``wheel_stall``. These come from
    ``par_anomaly.anomaly_detector`` and represent IMU + odom-based
    detections that do NOT require operator annotation, unlike the
    legacy ``collision`` event which is operator-marked.
    """
    return sum(1 for r in rows if r.kind == "event" and r.name == event_name)


def _long_tof_clamp_events(rows: list[Row], min_s: float = 1.5) -> int:
    """Count contiguous tof-clamp runs in arbiter winner log lines lasting ≥ min_s."""
    runs = []
    run_start: float | None = None
    for r in rows:
        if r.kind != "log":
            continue
        if r.clamp_reason == "tof":
            if run_start is None:
                run_start = r.t_s
            else:
                run_end = r.t_s  # update as run continues
        else:
            if run_start is not None:
                # Run ended — run_end is the previous tof row's timestamp.
                # We close on first non-tof row; approximate run_end as r.t_s - 0.05.
                pass
            run_start = None
    # Simpler approach: collect tof-clamp timestamps, then find gaps > 0.5 s
    # (sample period ~50 ms) to define runs.
    tof_times = sorted(
        r.t_s for r in rows
        if r.kind == "log" and r.clamp_reason == "tof"
    )
    if not tof_times:
        return 0
    gap_threshold_s = 0.5
    runs = []
    run_start = tof_times[0]
    last_t = tof_times[0]
    for t in tof_times[1:]:
        if t - last_t > gap_threshold_s:
            runs.append((run_start, last_t))
            run_start = t
        last_t = t
    runs.append((run_start, last_t))
    return sum(1 for s, e in runs if (e - s) >= min_s)


def _recovery_success_rate(rows: list[Row]) -> tuple[float | None, int, int]:
    """Pair each DEAD_END_* intent with the next reactive/HSGR or reactive/FORWARD
    winner within 10 s. Return (rate, successful, total)."""
    dead_ends = [
        r for r in rows
        if r.kind == "intent" and r.name.startswith("DEAD_END")
    ]
    if not dead_ends:
        return None, 0, 0
    # Successful exits: a winner=reactive/HSGR or winner=reactive/FORWARD with v>0.10
    # within 10 s of the DEAD_END's timestamp.
    successful = 0
    SEEN_GAP_S = 10.0
    for de in dead_ends:
        for r in rows:
            if r.t_s < de.t_s:
                continue
            if r.t_s - de.t_s > SEEN_GAP_S:
                break
            if r.kind != "log":
                continue
            w = r.winner
            v = r.linear_x or 0.0
            if w and w[0] == "reactive" and w[1] in ("HSGR", "FORWARD") and v > 0.10:
                successful += 1
                break
    total = len(dead_ends)
    rate = successful / total if total else None
    return rate, successful, total


def _dynamic_response_latency(rows: list[Row]) -> float | None:
    """First operator dyn_entered annotation → first non-cruise winner after it.
    Returns latency in seconds, or None if no annotation."""
    dyn_event = next(
        (r for r in rows if r.kind == "event" and r.name == "dyn_entered"),
        None,
    )
    if dyn_event is None:
        return None
    # First subsequent winner with label NOT in {HSGR, FORWARD} = the robot
    # reacted (deflected, slowed, stopped).
    for r in rows:
        if r.t_s <= dyn_event.t_s:
            continue
        if r.kind != "log":
            continue
        w = r.winner
        if w and w[0] == "reactive" and w[1] not in ("HSGR", "FORWARD"):
            return r.t_s - dyn_event.t_s
    return None


def _wedge_dwell_times(rows: list[Row]) -> list[float]:
    """For each DEAD_END_* intent, find the first subsequent reactive/HSGR or FORWARD
    winner with v>0.10 and report (exit_time - dead_end_time)."""
    dwells: list[float] = []
    for de in (r for r in rows if r.kind == "intent" and r.name.startswith("DEAD_END")):
        for r in rows:
            if r.t_s <= de.t_s:
                continue
            if r.kind != "log":
                continue
            w = r.winner
            v = r.linear_x or 0.0
            if w and w[0] == "reactive" and w[1] in ("HSGR", "FORWARD") and v > 0.10:
                dwells.append(r.t_s - de.t_s)
                break
    return dwells


def _algo_state_distribution(rows: list[Row]) -> dict[str, float]:
    """Histogram of winner=reactive/<label> over all arbiter log rows. Returns a
    fractions dict (sum to 1.0)."""
    labels: list[str] = []
    for r in rows:
        if r.kind != "log":
            continue
        w = r.winner
        if w and w[0] == "reactive":
            labels.append(w[1])
    if not labels:
        return {}
    counts = Counter(labels)
    total = sum(counts.values())
    return {label: count / total for label, count in counts.most_common()}


def _path_length_m(rows: list[Row]) -> float:
    """Sum Euclidean distance between consecutive odom samples."""
    odom_points = [r.odom_xy for r in rows if r.kind == "odom" and r.odom_xy is not None]
    if len(odom_points) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(odom_points, odom_points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _coverage_m2_per_min(rows: list[Row], duration_min: float,
                          chassis_width_m: float = 0.33) -> float | None:
    """Approximate swept area / minute. Path length × chassis width is an
    overestimate (overlapping passes count multiple times) but rubric-acceptable
    as a proxy. Returns None when duration ≤ 0 or no odom samples."""
    if duration_min <= 0:
        return None
    path = _path_length_m(rows)
    if path <= 0:
        return None
    return (path * chassis_width_m) / duration_min


def _percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def compute_metrics(rows: list[Row], config: dict[str, Any]) -> dict[str, Any]:
    start_t, end_t = _trial_window(rows)
    duration_s = max(0.0, end_t - start_t)
    duration_min = duration_s / 60.0 if duration_s > 0 else 0.0

    annotated_collisions = _collision_count(rows)
    long_tof_events = _long_tof_clamp_events(rows)
    # par_anomaly ground-truth detections — no operator annotation.
    tilt_count = _anomaly_event_count(rows, "tilt")
    collision_impact_count = _anomaly_event_count(rows, "collision_impact")
    wheel_stall_count = _anomaly_event_count(rows, "wheel_stall")
    wheel_impact_count = _anomaly_event_count(rows, "wheel_impact")
    # auto_collisions: union of contact-event types from par_anomaly. Used
    # as the primary collision metric for the integrated-course trials so
    # operator annotation is not required during a 5-min trial. wheel_stall
    # is excluded because sustained stall can occur without an obstacle
    # (mechanical bind, motor stall on slick floor), while jerk + impact
    # + tilt all imply contact.
    auto_collisions = collision_impact_count + wheel_impact_count + tilt_count
    if duration_min > 0:
        auto_collisions_per_min = auto_collisions / duration_min
    else:
        auto_collisions_per_min = None
    if duration_min > 0:
        collisions_per_min = (annotated_collisions + 0.5 * long_tof_events) / duration_min
    else:
        collisions_per_min = None

    recovery_rate, succ, total = _recovery_success_rate(rows)
    dyn_latency = _dynamic_response_latency(rows)
    dwells = _wedge_dwell_times(rows)
    distribution = _algo_state_distribution(rows)

    return {
        "trial_id": config.get("trial_id", "unknown"),
        "algo": config.get("algo", "unknown"),
        "use_depth": config.get("use_depth"),
        "tof_off": config.get("tof_off"),
        "detection_tier": config.get("detection_tier", "default"),
        "scenario": config.get("scenario", "unknown"),
        "duration_s": round(duration_s, 2),
        "annotated_collisions": annotated_collisions,
        "long_tof_clamp_events": long_tof_events,
        "collisions_per_min": (round(collisions_per_min, 3)
                                if collisions_per_min is not None else None),
        "recovery_success_rate": (round(recovery_rate, 3)
                                   if recovery_rate is not None else None),
        "recovery_successful_count": succ,
        "recovery_total_count": total,
        "dynamic_response_latency_s": (round(dyn_latency, 3)
                                        if dyn_latency is not None else None),
        "wedge_dwell_p50_s": (round(_percentile(dwells, 0.5), 3)
                               if dwells else None),
        "wedge_dwell_max_s": (round(max(dwells), 3) if dwells else None),
        "wedge_count": len(dwells),
        "algo_state_distribution": {
            k: round(v, 3) for k, v in distribution.items()
        },
        "coverage_m2_per_min": (round(_coverage_m2_per_min(rows, duration_min), 3)
                                  if _coverage_m2_per_min(rows, duration_min) is not None
                                  else None),
        "path_length_m": round(_path_length_m(rows), 3),
        # par_anomaly columns
        "tilt_count": tilt_count,
        "collision_impact_count": collision_impact_count,
        "wheel_stall_count": wheel_stall_count,
        "wheel_impact_count": wheel_impact_count,
        "auto_collisions_per_min": (round(auto_collisions_per_min, 3)
                                     if auto_collisions_per_min is not None
                                     else None),
    }


# --- Outputs ---------------------------------------------------------------

CSV_FIELDS = [
    "trial_id", "algo", "scenario", "use_depth", "detection_tier", "duration_s",
    "annotated_collisions", "collisions_per_min",
    "recovery_success_rate", "recovery_successful_count", "recovery_total_count",
    "dynamic_response_latency_s", "wedge_dwell_p50_s", "wedge_dwell_max_s",
    "wedge_count", "coverage_m2_per_min",
    # par_anomaly columns
    "tilt_count", "collision_impact_count", "wheel_stall_count", "wheel_impact_count",
    "auto_collisions_per_min",
]


def seed_debrief(metrics: dict[str, Any], config: dict[str, Any]) -> str:
    return f"""# Trial debrief — {metrics['trial_id']}

**Scenario**: {metrics['scenario']}  •  **Algorithm**: {metrics['algo']}  •
**Sensors**: {'LIDAR+depth' if metrics.get('use_depth') else 'LIDAR-only'}  •
**Detection tier**: {metrics.get('detection_tier', 'default')}

## Auto-extracted metrics
- Duration: {metrics['duration_s']:.1f} s
- Annotated collisions: {metrics['annotated_collisions']}
- Collisions/min: {metrics.get('collisions_per_min')}
- Recovery success rate: {metrics.get('recovery_success_rate')} ({metrics['recovery_successful_count']}/{metrics['recovery_total_count']})
- Dynamic response latency: {metrics.get('dynamic_response_latency_s')} s
- Wedge dwell p50/max: {metrics.get('wedge_dwell_p50_s')} / {metrics.get('wedge_dwell_max_s')} s ({metrics['wedge_count']} wedges)
- Algorithm state distribution:
{chr(10).join(f"  - {k}: {v:.0%}" for k, v in metrics['algo_state_distribution'].items())}

## Operator notes (fill in)

### What did the robot do well?

### What did the robot do poorly?

### Surprises / out-of-envelope events?

### Re-run? (yes/no, why)
"""


def append_csv_row(csv_path: pathlib.Path, metrics: dict[str, Any]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = csv_path.exists()
    with csv_path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        if not file_exists:
            writer.writeheader()
        writer.writerow({k: metrics.get(k) for k in CSV_FIELDS})


def _safe_yaml_dump(d: dict[str, Any]) -> str:
    """Minimal YAML serialiser — avoids the PyYAML dep on the operator Mac."""
    lines = []
    for k, v in d.items():
        if isinstance(v, dict):
            lines.append(f"{k}:")
            for sk, sv in v.items():
                lines.append(f"  {sk}: {sv}")
        elif v is None:
            lines.append(f"{k}: null")
        elif isinstance(v, bool):
            lines.append(f"{k}: {'true' if v else 'false'}")
        elif isinstance(v, str):
            lines.append(f"{k}: \"{v}\"")
        else:
            lines.append(f"{k}: {v}")
    return "\n".join(lines) + "\n"


def _safe_yaml_load(text: str) -> dict[str, Any]:
    """Minimal YAML loader — handles the small subset run_trial.sh writes."""
    out: dict[str, Any] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].rstrip()
        if not line or ":" not in line or line.startswith(" "):
            continue
        k, v = line.split(":", 1)
        v = v.strip()
        if v.startswith('"') and v.endswith('"'):
            out[k.strip()] = v[1:-1]
        elif v.lower() in ("true", "false"):
            out[k.strip()] = v.lower() == "true"
        elif v == "null" or v == "":
            out[k.strip()] = None
        else:
            try:
                out[k.strip()] = float(v) if "." in v else int(v)
            except ValueError:
                out[k.strip()] = v
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyse a PAR-A3 trial session.")
    parser.add_argument("session_dir", type=pathlib.Path)
    parser.add_argument("--csv", type=pathlib.Path,
                        default=pathlib.Path("report/data/trials.csv"))
    parser.add_argument("--window-s", type=float, default=None,
                        help="If set, slice rows to a fixed drive window "
                             "[first_arbiter_switch, +window_s] before computing "
                             "metrics. Use 150 to align with the standardised "
                             "trial drive duration. Suffix output keys with the "
                             "window for traceability.")
    args = parser.parse_args()

    log_path = args.session_dir / "log.txt"
    if not log_path.exists():
        print(f"ERROR: {log_path} not found", file=sys.stderr)
        return 2

    config_path = args.session_dir / "trial_config.yaml"
    config = (_safe_yaml_load(config_path.read_text())
              if config_path.exists() else {})

    rows = parse_log_lines(log_path.read_text())
    if args.window_s is not None:
        rows = _slice_to_drive_window(rows, args.window_s)
    metrics = compute_metrics(rows, config)
    if args.window_s is not None:
        metrics["window_s"] = args.window_s

    metrics_path = args.session_dir / "metrics.yaml"
    metrics_path.write_text(_safe_yaml_dump(metrics))

    debrief_path = args.session_dir / "debrief.md"
    if not debrief_path.exists():
        debrief_path.write_text(seed_debrief(metrics, config))

    append_csv_row(args.csv, metrics)

    # Print summary. auto_collisions_per_min is the canonical metric
    # (tilt + collision_impact + wheel_impact / minute). The operator-
    # annotated collisions_per_min is shown alongside for context but is
    # 0 unless annotate.sh was used live. Verdict threshold applies to
    # the auto metric.
    auto_cpm = metrics.get("auto_collisions_per_min")
    annot_cpm = metrics.get("collisions_per_min")
    verdict = "GOOD"
    if metrics["duration_s"] < 60:
        verdict = "RE-RUN (too short)"
    elif auto_cpm is not None and auto_cpm > 0.5:
        verdict = "RE-RUN (high collision rate)"
    print(f"trial_id={metrics['trial_id']} duration={metrics['duration_s']:.1f}s "
          f"auto_collisions/min={auto_cpm} "
          f"(tilt={metrics.get('tilt_count')} "
          f"impact={metrics.get('collision_impact_count')} "
          f"wheel_imp={metrics.get('wheel_impact_count')}) "
          f"annotated/min={annot_cpm} "
          f"recovery={metrics.get('recovery_success_rate')} "
          f"verdict={verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
