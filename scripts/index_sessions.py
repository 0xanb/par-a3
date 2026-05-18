#!/usr/bin/env python3
"""Build `report/data/sessions/` — a trial-indexed view of `logs/`.

For each `logs/session_<stamp>/trial_config.yaml`, read trial_id and produce:
 report/data/sessions/<trial_id>/ -> symlink to ///logs/session_<stamp>/

Also writes report/data/sessions/manifest.csv with columns:
 trial_id, session_dir, system_state, duration_s, tilt, impact, wheel_imp,
 auto_collisions_per_min, verdict, notes

system_state is derived from the trial_id sequence and commit history:
- pre-F01: anything before commit 4e9e761 (rear-ToF wiring)
- post-F01-pre-F08: between 4e9e761 and 50b75af
- post-F08: 50b75af onward (depth aligned)

We approximate by trial_id sequence ordering since timestamps are noisy:
 _0104 (CAL series + early T-series): pre-F01
 _0507 (T-04 + lidar-only T-05/T-06): post-F01-pre-F08
 _08+: post-F08

Idempotent: re-running replaces stale symlinks and rewrites manifest.csv.
"""
from __future__ import annotations
import csv
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve.parent.parent
LOGS = ROOT / "logs"
OUT = ROOT / "report" / "data" / "sessions"


def parse_yaml_minimal(path: pathlib.Path) -> dict:
 """Tiny YAML reader for the trial_config schema we control."""
 out: dict = {}
 if not path.exists:
 return out
 in_notes = False
 notes_lines: list[str] = []
 for raw in path.read_text.splitlines:
 line = raw.rstrip
 if in_notes:
 if line.startswith(" ") or line.startswith("\t"):
 notes_lines.append(line.lstrip)
 continue
 else:
 out["notes"] = "\n".join(notes_lines).strip
 in_notes = False
 m = re.match(r"^([a-zA-Z_][\w]*):\s*(.*)$", line)
 if not m:
 continue
 key, val = m.group(1), m.group(2).strip
 if key == "notes" and val == "|":
 in_notes = True
 continue
 # strip surrounding quotes
 if val.startswith('"') and val.endswith('"'):
 val = val[1:-1]
 out[key] = val
 if in_notes and notes_lines:
 out["notes"] = "\n".join(notes_lines).strip
 return out


def parse_metrics(path: pathlib.Path) -> dict:
 return parse_yaml_minimal(path)


def system_state_for(trial_id: str) -> str:
 """Map trial_id to system state. The seq is the trailing _NN."""
 m = re.search(r"_(\d{2})$", trial_id)
 if not m:
 return "unknown"
 seq = int(m.group(1))
 # Integrated lidar+depth cell — most populated cell, easy partition:
 if "integrated_nd_hybrid_lidar_depth" in trial_id:
 if seq <= 4:
 return "pre-F01"
 if seq <= 7:
 return "post-F01-pre-F08"
 return "post-F08"
 # All other cells were run :
 if "lidar_only" in trial_id or "vfh_plus" in trial_id or "nd_only" in trial_id:
 # Determined per the actual date the trial ran; conservative default:
 return "post-F01-pre-F08"
 if "tilt_recovery" in trial_id or "tof_off_safety" in trial_id:
 return "pre-F01" # CAL series
 return "unknown"


def main -> int:
 if not LOGS.exists:
 print(f"missing {LOGS}", file=sys.stderr)
 return 2
 OUT.mkdir(parents=True, exist_ok=True)

 # Clear stale symlinks (but keep manifest.csv until we rewrite).
 for child in OUT.iterdir:
 if child.is_symlink:
 child.unlink

 rows: list[dict] = []
 for session in sorted(LOGS.glob("session_*")):
 config = session / "trial_config.yaml"
 if not config.exists:
 continue
 cfg = parse_yaml_minimal(config)
 trial_id = cfg.get("trial_id")
 if not trial_id:
 continue
 # Create symlink trial_id -> session
 link = OUT / trial_id
 if link.exists or link.is_symlink:
 link.unlink
 rel = pathlib.Path("") / "" / "" / "logs" / session.name
 link.symlink_to(rel)

 metrics = parse_metrics(session / "metrics.yaml")
 rows.append({
 "trial_id": trial_id,
 "session_dir": session.name,
 "system_state": system_state_for(trial_id),
 "scenario": cfg.get("scenario", ""),
 "algo": cfg.get("algo", ""),
 "sensor": cfg.get("sensor", ""),
 "use_depth": cfg.get("use_depth", ""),
 "tof_off": cfg.get("tof_off", ""),
 "duration_s": metrics.get("duration_s", ""),
 "tilt_count": metrics.get("tilt_count", ""),
 "collision_impact_count": metrics.get("collision_impact_count", ""),
 "wheel_impact_count": metrics.get("wheel_impact_count", ""),
 "auto_collisions_per_min": metrics.get("auto_collisions_per_min", ""),
 "recovery_success_rate": metrics.get("recovery_success_rate", ""),
 "wedge_count": metrics.get("wedge_count", ""),
 "notes_preview": (cfg.get("notes", "")[:80] + "") if cfg.get("notes") else "",
 })

 # Sort: pre-F01 → post-F01-pre-F08 → post-F08 → unknown; then by trial_id.
 order = {"pre-F01": 0, "post-F01-pre-F08": 1, "post-F08": 2, "unknown": 3}
 rows.sort(key=lambda r: (order.get(r["system_state"], 9), r["trial_id"]))

 manifest = OUT / "manifest.csv"
 with manifest.open("w", newline="") as fh:
 writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys) if rows else ["trial_id"])
 writer.writeheader
 for r in rows:
 writer.writerow(r)

 print(f"wrote {manifest}")
 print(f"linked {len(rows)} sessions under {OUT}")
 print
 print("Sessions by system state:")
 counts: dict = {}
 for r in rows:
 counts[r["system_state"]] = counts.get(r["system_state"], 0) + 1
 for state, n in counts.items:
 print(f" {state}: {n}")
 return 0


if __name__ == "__main__":
 sys.exit(main)
