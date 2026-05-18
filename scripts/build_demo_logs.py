#!/usr/bin/env python3
"""Build the official `demo_logs/` folder from the keeper trials.

Each keeper gets its own subdirectory with the full session contents
(trial_config.yaml, metrics.yaml, log.txt, captures/, path.png, path.csv,
debrief.md). The aggregate view at the root has README.md, all_trials.csv,
aligned_paths_grid.png, and aligned_paths_overlay.png — all paths share
the (1, 1) start so cross-cell comparison is direct.

Usage:
 python3 scripts/build_demo_logs.py
"""
from __future__ import annotations
import csv
import os
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve.parent.parent
DEMO = ROOT / "report" / "data" / "demo_logs"
SESS_DIR = ROOT / "report" / "data" / "sessions"

# Keeper trial-ids — the ones that go into §4 of the report.
# Selection rationale per cell:
# - nd_hybrid + lidar+depth (headline, pre-): _08,09,10 = T-04d/e/f
# (headline). Awaiting T-04g/h after charge.
# - nd_hybrid + lidar-only TRUE: _03,04 = T-05b/T-06b
# - nd_hybrid + depth-only NEW: _01,02 = T-12a/b
# - nd_hybrid + tof_off + v_max=0.05: _03,04 = T-11b/T-11c ( 150 s)
# - nd_only + lidar+depth: _01,02 = T-09/T-10
# - vfh_plus + lidar+depth (pre-): _01,02 = T-07/T-08
KEEPERS = [
 # cell trial_id label
 ("nd_hybrid_lidar+depth_pre_B11", "C_integrated_nd_hybrid_lidar_depth_08", "T-04d (headline)"),
 ("nd_hybrid_lidar+depth_pre_B11", "C_integrated_nd_hybrid_lidar_depth_09", "T-04e (headline)"),
 ("nd_hybrid_lidar+depth_pre_B11", "C_integrated_nd_hybrid_lidar_depth_10", "T-04f (headline)"),
 ("nd_only_lidar+depth", "C_integrated_nd_only_lidar_depth_01", "T-09"),
 ("nd_only_lidar+depth", "C_integrated_nd_only_lidar_depth_02", "T-10"),
 ("vfh_plus_lidar+depth_pre_B11", "C_integrated_vfh_plus_lidar_depth_01", "T-07"),
 ("vfh_plus_lidar+depth_pre_B11", "C_integrated_vfh_plus_lidar_depth_02", "T-08"),
 ("nd_hybrid_lidar_only_TRUE", "C_integrated_nd_hybrid_lidar_only_03", "T-05b "),
 ("nd_hybrid_lidar_only_TRUE", "C_integrated_nd_hybrid_lidar_only_04", "T-06b "),
 ("nd_hybrid_depth_only_NEW", "C_integrated_nd_hybrid_depth_only_01", "T-12a "),
 ("nd_hybrid_depth_only_NEW", "C_integrated_nd_hybrid_depth_only_02", "T-12b "),
 ("nd_hybrid_tof_off_safety", "C_tof_off_safety_nd_hybrid_lidar_depth_03", "T-11b (150 s)"),
 ("nd_hybrid_tof_off_safety", "C_tof_off_safety_nd_hybrid_lidar_depth_04", "T-11c"),
]


def copy_session(trial_id: str, dst: pathlib.Path) -> bool:
 src = (SESS_DIR / trial_id).resolve
 if not src.exists:
 print(f" SKIP {trial_id}: source not found at {src}", file=sys.stderr)
 return False
 if dst.exists:
 shutil.rmtree(dst)
 dst.mkdir(parents=True)
 for item in src.iterdir:
 if item.is_dir:
 shutil.copytree(item, dst / item.name)
 else:
 shutil.copy2(item, dst / item.name)
 return True


def main -> int:
 DEMO.mkdir(exist_ok=True)
 summary_rows = []
 cells: dict[str, list[str]] = {}

 for cell, trial_id, label in KEEPERS:
 print(f"copying {trial_id} -> demo_logs/{cell}/{trial_id}")
 dst = DEMO / cell / trial_id
 if not copy_session(trial_id, dst):
 continue
 cells.setdefault(cell, []).append(trial_id)

 # Re-run analyzer to produce metrics.yaml at the destination
 metrics_path = dst / "metrics.yaml"
 if not metrics_path.exists:
 try:
 subprocess.run([sys.executable, str(ROOT / "scripts/analyze_trial.py"),
 str(dst), "--window-s", "150"],
 check=True, capture_output=True)
 except subprocess.CalledProcessError as e:
 print(f" analyze_trial failed for {trial_id}: {e.stderr.decode[:200]}",
 file=sys.stderr)

 # Per-trial path plot
 try:
 subprocess.run([sys.executable, str(ROOT / "scripts/plot_path.py"),
 str(dst), "--window-s", "150"],
 check=True, capture_output=True)
 except subprocess.CalledProcessError as e:
 print(f" plot_path failed for {trial_id}: {e.stderr.decode[:200]}",
 file=sys.stderr)

 # Append a row to the aggregate CSV
 metrics: dict = {}
 if metrics_path.exists:
 for line in metrics_path.read_text.splitlines:
 if ":" in line:
 k, _, v = line.partition(":")
 metrics[k.strip] = v.strip
 summary_rows.append({
 "cell": cell,
 "trial_id": trial_id,
 "label": label,
 "duration_s": metrics.get("duration_s", ""),
 "tilt": metrics.get("tilt_count", ""),
 "impact": metrics.get("collision_impact_count", ""),
 "wheel_imp": metrics.get("wheel_impact_count", ""),
 "wheel_stall": metrics.get("wheel_stall_count", ""),
 "auto_per_min": metrics.get("auto_collisions_per_min", ""),
 "recovery": metrics.get("recovery_success_rate", ""),
 })

 csv_path = DEMO / "all_trials.csv"
 if summary_rows:
 with csv_path.open("w", newline="") as fh:
 w = csv.DictWriter(fh, fieldnames=list(summary_rows[0].keys))
 w.writeheader
 for r in summary_rows:
 w.writerow(r)
 print(f"wrote {csv_path}")

 # Aggregate path views: re-run plot_aligned_paths.py over the demo set
 # by temporarily symlinking the demo trials into a side dir the plotter
 # can consume. Simplest: just point the plotter at demo_logs/*/<trial_id>
 # by passing through via SESS_DIR symlinks (which it already reads).
 # We've already populated those, so a single invocation works.
 try:
 # Pass canonical trial IDs via TRIALS env var so the plotter
 # filters out contaminated / exploratory sessions in the index.
 canonical_ids = [r["trial_id"] for r in summary_rows]
 env = {**os.environ, "TRIALS": ",".join(canonical_ids)}
 subprocess.run([sys.executable, str(ROOT / "scripts/plot_aligned_paths.py")],
 check=True, env=env)
 # Copy the produced PNGs into demo_logs/
 for src_name in ("aligned_paths_grid.png", "aligned_paths_overlay.png"):
 src_p = ROOT / "report" / "data" / src_name
 if src_p.exists:
 shutil.copy2(src_p, DEMO / src_name)
 print(f"copied {src_name} -> demo_logs/")
 except subprocess.CalledProcessError as e:
 print(f"plot_aligned_paths failed: {e}", file=sys.stderr)

 # README
 readme = DEMO / "README.md"
 lines = []
 lines.append("# demo_logs — Official trial dataset for the §4 report")
 lines.append("")
 lines.append(f"Generated {pathlib.Path(__file__).name} on. Universal protocol: **150 s drive window, N ≥ 2 replicates per cell**. Paths aligned in `aligned_paths_grid.png` and `aligned_paths_overlay.png` with start = (1, 1) in arena frame, dead-end at (0, 0).")
 lines.append("")
 lines.append("## Per-cell results")
 lines.append("")
 lines.append("| Cell | Trials | N | mean auto/min |")
 lines.append("|---|---|---:|---:|")
 cell_means = []
 for cell, trial_ids in cells.items:
 rates = []
 for r in summary_rows:
 if r["cell"] == cell and r["auto_per_min"]:
 try:
 rates.append(float(r["auto_per_min"]))
 except ValueError:
 pass
 mean = (sum(rates) / len(rates)) if rates else float("nan")
 cell_means.append((cell, mean))
 lines.append(f"| {cell} | {', '.join(trial_ids)} | {len(rates)} | {mean:.2f} |")
 lines.append("")
 lines.append("## Trial-by-trial CSV")
 lines.append("")
 lines.append("See `all_trials.csv`. Columns: cell, trial_id, label, duration_s, tilt, impact, wheel_imp, wheel_stall, auto_per_min, recovery.")
 lines.append("")
 lines.append("## Per-trial folder contents")
 lines.append("")
 lines.append("Each `demo_logs/<cell>/<trial_id>/` contains:")
 lines.append("- `trial_config.yaml` — algo/sensor/duration/notes for this trial")
 lines.append("- `metrics.yaml` — analyzer output (tilt/impact/wheel/recovery counts)")
 lines.append("- `log.txt` — raw text log (intents, events, mode, odom pose)")
 lines.append("- `captures/` — snapshotter frames at each anomaly event")
 lines.append("- `path.png` — per-trial path plot (color = time, markers = events)")
 lines.append("- `path.csv` — t_s, x, y for further analysis")
 lines.append("- `debrief.md` — operator notes")
 lines.append("")
 lines.append("## Aggregate views")
 lines.append("")
 lines.append("- `aligned_paths_grid.png` — per-trial path panels in arena frame (start (1,1), heading +y)")
 lines.append("- `aligned_paths_overlay.png` — all trial paths on one chart with obstacle layout")
 lines.append("- `all_trials.csv` — flat table of every keeper trial's metrics")
 readme.write_text("\n".join(lines) + "\n")
 print(f"wrote {readme}")

 print
 print(f"DONE: {len(summary_rows)} keeper trials under {DEMO}")
 print
 print("Cell ranking (mean auto_collisions/min, lower is better):")
 for cell, mean in sorted(cell_means, key=lambda x: x[1] if x[1] == x[1] else 999):
 print(f" {mean:5.2f} {cell}")
 return 0


if __name__ == "__main__":
 sys.exit(main)
