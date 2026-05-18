#!/usr/bin/env python3
"""Render every trial-id path translated to a common start point at (0,0)
and rotated so initial heading is +x. Operator placed the robot at the
same physical spot for every trial, so aligned paths are directly
comparable obstacle-by-obstacle.

Outputs:
 report/data/aligned_paths_grid.png — per-trial panels
 report/data/aligned_paths_overlay.png — all paths overlaid on one axes
 report/data/aligned_paths.csv — t_s, trial_id, x_aligned, y_aligned
"""
from __future__ import annotations
import csv
import math
import pathlib
import re
import sys
from datetime import datetime

try:
 import matplotlib
 matplotlib.use("Agg")
 import matplotlib.pyplot as plt
except ImportError:
 print("matplotlib required", file=sys.stderr); sys.exit(2)

ROOT = pathlib.Path(__file__).resolve.parent.parent
SESS_DIR = ROOT / "report" / "data" / "sessions"

ODOM_RE = re.compile(
 r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+odom\s+-\s+pose\s+x=(-?\d+\.\d+)\s+y=(-?\d+\.\d+)"
)
EVENT_RE = re.compile(
 r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+event\s+\S+\s+(\S+)"
)
MODE_RE = re.compile(
 r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+mode\s+\S+\s+(\S+)\s+reason"
)


def parse_ts(s: str) -> float:
 return datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f").timestamp


def parse_session(log: pathlib.Path, window_s: float = 150.0):
 odom, events = [], []
 drive_start = None
 t0 = None
 for line in log.read_text(errors="replace").splitlines:
 m = ODOM_RE.match(line)
 if m:
 t = parse_ts(m.group(1))
 t0 = t if t0 is None else t0
 odom.append((t - t0, float(m.group(2)), float(m.group(3))))
 continue
 m = EVENT_RE.match(line)
 if m:
 t = parse_ts(m.group(1))
 t0 = t if t0 is None else t0
 events.append((t - t0, m.group(2)))
 continue
 m = MODE_RE.match(line)
 if m and m.group(2) == "A" and drive_start is None:
 t = parse_ts(m.group(1))
 t0 = t if t0 is None else t0
 drive_start = t - t0
 if drive_start is not None:
 t_lo, t_hi = drive_start, drive_start + window_s
 odom = [r for r in odom if t_lo <= r[0] <= t_hi]
 events = [e for e in events if t_lo <= e[0] <= t_hi]
 return odom, events


# Arena physical layout (operator-supplied, supersedes 3 × 3 estimate):
# Arena = 4.5 m (x) × 3.5 m (y), rectangular. Dead-end at bottom-left corner = (0, 0).
# Robot start position = (1, 1). Initial heading +y (toward the top wall).
# Open door at top-right corner (x ≈ 4, y ≈ 3.5).
ARENA_WIDTH_M = 4.5 # x extent
ARENA_HEIGHT_M = 3.0 # y extent (operator-measured rev 4: anything beyond y=3 is outside the room)
START_X_M = 1.0
START_Y_M = 1.0
START_HEADING_RAD = math.pi / 2 # +y direction

# Static obstacles drawn at their TRUE physical footprint (rectangles + circles),
# operator-supplied spec. Each obstacle is its real shape and size, not
# a marker glyph. All grey-toned and low-alpha so path traces stay readable.
#
# Schema (one entry per obstacle):
# ("rect", cx, cy, w, h, label) — centred rectangle of width w × height h
# ("circle", cx, cy, r, label) — circle of radius r centred at (cx, cy)
# ("rectc", x0, y0, w, h, label) — corner-anchored rectangle from (x0,y0) extent (w,h)
#
# Sizes are best-effort physical footprints; the exact values come from the
# operator spec where given, and reasonable household-furniture defaults
# (chair 0.4×0.4, small box 0.2×0.2, foam roller 0.15×0.6, fan base 0.3 dia)
# elsewhere.
_OBS_FACE = "#B5B5B5"
_OBS_EDGE = "#666666"
_OBS_ALPHA = 0.45

OBSTACLES = [
 # Operator-supplied (arena_editor.html export, rev 4).
 # Arena y-axis cropped to 3.0 m — beyond that is outside the room.
 # All centres (cx, cy) and dimensions (w, h or radius) in arena metres.
 ("circle", 1.07, 0.350, 0.307, "fan"),
 ("circle", 0.57, 2.020, 0.135, "roller"),
 ("rect", 1.515, 1.080, 0.20, 0.20, "box"),
 ("rect", 1.525, 1.530, 0.20, 0.20, "box"),
 ("rect", 0.695, 2.930, 0.46, 0.16, "box(wall)"),
 ("rect", 3.145, 1.510, 2.285, 1.045, "table"),
 ("rect", 3.225, 0.550, 0.63, 0.59, "chair"),
 ("rect", 4.170, 1.105, 0.545, 2.140, "chair(table)"),
 ("rect", 2.130, 2.765, 0.97, 0.51, "rug"),
 ("rect", 2.840, 2.625, 0.16, 0.42, "slippers"),
 ("rect", 4.050, 2.430, 0.30, 0.30, "backpack"),
 ("rect", 3.500, 2.930, 1.07, 0.20, "door"),
]


def draw_arena(ax, show_obstacles=True):
 """Draw the rectangular arena, dead-end marker, and obstacles at their
 true physical footprint (rectangles and circles, not marker glyphs)."""
 ax.add_patch(plt.Rectangle((0, 0), ARENA_WIDTH_M, ARENA_HEIGHT_M,
 fill=False, edgecolor="black", linewidth=1.5,
 zorder=0))
 ax.scatter(0, 0, color="black", s=180, marker="X", zorder=2)
 ax.text(0.05, 0.05, "dead-end", fontsize=7, color="black")
 if not show_obstacles:
 return
 for spec in OBSTACLES:
 kind = spec[0]
 if kind == "rect":
 _, cx, cy, w, h, label = spec
 x0, y0 = cx - w / 2, cy - h / 2
 ax.add_patch(plt.Rectangle((x0, y0), w, h, facecolor=_OBS_FACE,
 edgecolor=_OBS_EDGE, linewidth=0.5,
 alpha=_OBS_ALPHA, zorder=1))
 tx, ty = cx, cy
 elif kind == "rectc":
 _, x0, y0, w, h, label = spec
 ax.add_patch(plt.Rectangle((x0, y0), w, h, facecolor=_OBS_FACE,
 edgecolor=_OBS_EDGE, linewidth=0.5,
 alpha=_OBS_ALPHA, zorder=1))
 tx, ty = x0 + w / 2, y0 + h / 2
 elif kind == "circle":
 _, cx, cy, r, label = spec
 ax.add_patch(plt.Circle((cx, cy), r, facecolor=_OBS_FACE,
 edgecolor=_OBS_EDGE, linewidth=0.5,
 alpha=_OBS_ALPHA, zorder=1))
 tx, ty = cx, cy
 else:
 continue
 ax.text(tx, ty, label, fontsize=6, color="#222222", ha="center",
 va="center", weight="bold", zorder=6,
 bbox=dict(facecolor="white", edgecolor="none",
 alpha=0.65, pad=0.6))


# Event-anchored refinement targets (in arena frame). Operator observation
#: across the canonical trials, the obstacles that take the most
# hits cluster at the fan (close to start + dead-end → robot re-hits it
# during recovery → MULTIPLE events) and the slippers/rug/chairs (one-shot
# hits → SINGLE events). Other obstacles absorb some hits but at much
# lower rate. We use these as rotational anchors and choose the weighting
# based on the event count for the trial.
EVENT_HOTSPOTS_MULTI = [ # 3+ events: fan-dominated (recovery loops near dead-end)
 (1.07, 0.35, 6.0), # fan
 (2.87, 2.77, 3.0), # slippers
 (3.23, 0.73, 2.5), # chair (right)
 (2.16, 3.10, 2.5), # rug
 (4.35, 1.25, 2.0), # chair(table)
]
EVENT_HOTSPOTS_FEW = [ # 1-2 events: easier-to-handle obstacles
 (2.87, 2.77, 5.0), # slippers
 (2.16, 3.10, 4.5), # rug
 (3.23, 0.73, 4.0), # chair (right)
 (4.35, 1.25, 3.5), # chair(table)
 (1.07, 0.35, 2.5), # fan
]

# Per-trial manual rotation overrides (degrees, CCW). Applied AFTER the
# auto-refinement when operator inspection finds a residual rotation error.
# Operator notes:
# _09: actually exited through the open door (top-right), so we need to
# rotate ~+45° to align the east-going tail with the door at y≈3.4
# depth_only_02: the single tilt likely happened at the rug, not the
# chair; rotate ~+60° to lift the tilt-position from y≈1 to y≈3
TRIAL_ROTATION_OVERRIDES_DEG = {
 # Operator-tuned via `tools/arena_editor.html` tab ② "Path
 # realignment". Each value is the rotation (deg CCW around start) that
 # the operator dialled in to align the trial's path + events with the
 # actual physical obstacles hit (slippers / fan / chairs / rug).
 "C_integrated_nd_hybrid_depth_only_01": 104.0,
 "C_integrated_nd_hybrid_depth_only_02": 102.0,
 "C_integrated_nd_hybrid_lidar_depth_08": 39.0,
 "C_integrated_nd_hybrid_lidar_depth_09": 44.0,
 "C_integrated_nd_hybrid_lidar_only_03": 62.0,
 "C_tof_off_safety_nd_hybrid_lidar_depth_03": -81.0,
}


def _refine_rotation_by_events(odom_aligned, events, theta_initial):
 """Search ±90° around theta_initial for a rotational nudge that puts
 event positions closest to known collision hotspots. Returns the
 refined rotation angle (rad)."""
 if not events:
 return 0.0
 # Compute event positions in current aligned frame
 ev_xy = []
 first_t = events[0][0]
 for et, ev in events:
 if ev not in ("tilt", "collision_impact", "wheel_impact"):
 continue
 # interpolate path position at event time
 x, y = None, None
 for t, xa, ya in odom_aligned:
 if t >= et:
 x, y = xa, ya; break
 if x is None and odom_aligned:
 x, y = odom_aligned[-1][1], odom_aligned[-1][2]
 if x is not None:
 ev_xy.append((et - first_t, x, y))
 if not ev_xy:
 return 0.0

 sx, sy = START_X_M, START_Y_M
 # Pick the hotspot weighting based on event count: many events → trial
 # got stuck near the dead-end / fan and re-hit the same obstacle during
 # recovery; few events → robot navigated to an easier-to-handle target
 # (slippers, rug, chair) on a one-shot collision.
 hotspots = EVENT_HOTSPOTS_MULTI if len(ev_xy) >= 3 else EVENT_HOTSPOTS_FEW

 def cost(rot_rad):
 cr, sr = math.cos(rot_rad), math.sin(rot_rad)
 total = 0.0
 for dt, ex, ey in ev_xy:
 dx, dy = ex - sx, ey - sy
 xr = dx * cr - dy * sr + sx
 yr = dx * sr + dy * cr + sy
 best = None
 for hx, hy, w in hotspots:
 d2 = (xr - hx) ** 2 + (yr - hy) ** 2
 score = d2 / w
 if best is None or score < best:
 best = score
 total += best
 return total

 best_rot = 0.0
 best_cost = cost(0.0)
 # Coarse sweep
 for deg in range(-90, 91, 5):
 rot = math.radians(deg)
 c = cost(rot)
 if c < best_cost:
 best_cost = c; best_rot = rot
 # Fine sweep around the coarse best
 coarse = math.degrees(best_rot)
 for ddeg in range(-5, 6):
 rot = math.radians(coarse + ddeg)
 c = cost(rot)
 if c < best_cost:
 best_cost = c; best_rot = rot
 return best_rot


def align(odom, chord_distance_m=1.5, events=None):
 """Translate every path so the robot's start = arena (START_X, START_Y),
 rotate so the chassis's initial heading aligns with +y. After alignment,
 every trial shares the same physical arena frame: (0, 0) is the dead-end
 corner, the operator-placed robot starts at (1, 1) facing +y.

 Heading estimation. Trace the path forward until cumulative travel crosses
 ``chord_distance_m`` (default 1.5 m), then use the chord (start → that
 point) bearing. At 1.5 m the chassis has cleared the planner's
 initial-tick yaw — operator-observed at trial setup, the auto-yaw of
 `vfh_plus` can swing the chassis 90°+ in the first 1–2 s before settling
 on the chosen valley. Estimating heading from a window shorter than that
 window puts the rotation into the auto-yaw direction (which is biased
 toward whichever side of the start position the planner's first valley
 happened to lie on), not the operator-placed heading.

 Post-rotation sanity check. If the centroid of the aligned path lands
 south of the start (i.e. behind the operator-placed +y), flip the
 rotation 180°. This catches trials where the auto-yaw drove the chord
 estimate into the rear hemisphere. Without this, the path renders with
 its mass below y=1 and visually "escapes" through the dead-end corner —
 a known false-negative of pure chord-bearing on rectangular arenas
 where the operator-placed heading is +y by construction.

 Wedge-mode trials (path < chord_distance_m total) fall through to the
 last-sample chord; rotation accuracy is academic for sub-metre paths."""
 if not odom:
 return [], None
 x0, y0 = odom[0][1], odom[0][2]
 cum = 0.0
 chord_idx = None
 for i in range(1, len(odom)):
 cum += math.hypot(odom[i][1] - odom[i - 1][1],
 odom[i][2] - odom[i - 1][2])
 if cum >= chord_distance_m:
 chord_idx = i
 break
 if chord_idx is None:
 chord_idx = len(odom) - 1
 dx = odom[chord_idx][1] - x0
 dy = odom[chord_idx][2] - y0
 if math.hypot(dx, dy) < 1e-3:
 theta = 0.0
 else:
 theta = math.atan2(dy, dx)
 # Rotate odom-frame so initial heading -> world +x, then rotate by +90°
 # to put it on world +y (arena's vertical axis).
 def _apply(rot_rad):
 cos_t, sin_t = math.cos(rot_rad), math.sin(rot_rad)
 out = []
 for t, x, y in odom:
 dx, dy = x - x0, y - y0
 xa = dx * cos_t - dy * sin_t + START_X_M
 ya = dx * sin_t + dy * cos_t + START_Y_M
 out.append((t, xa, ya))
 return out

 rot = START_HEADING_RAD - theta
 aligned = _apply(rot)
 # Auto-yaw sanity check: if the chord estimate landed in the rear
 # hemisphere of the operator-placed +y heading, the aligned path will
 # have its mass behind the start (mean ya < START_Y_M). The operator
 # never placed the robot facing south, so this signature is the
 # initial-tick yaw biasing the chord; flip 180° to recover.
 mean_ya = sum(p[2] for p in aligned) / max(len(aligned), 1)
 if mean_ya < START_Y_M:
 rot = rot + math.pi
 aligned = _apply(rot)
 # Event-anchored refinement: if the trial has anomaly events, nudge
 # the rotation so the events sit close to the known collision hotspots
 # (fan / slippers / chairs). Per operator observation, the first event
 # in each trial typically lands at the fan (closest to start, hit
 # earliest when the chassis drifts south).
 if events:
 delta = _refine_rotation_by_events(aligned, events, rot)
 if abs(delta) > 0.01:
 aligned = _apply(rot + delta)
 rot += delta
 return aligned, theta


def xy_at(t_query, odom):
 if not odom:
 return None
 for t, x, y in odom:
 if t >= t_query:
 return (x, y)
 return (odom[-1][1], odom[-1][2])


def main -> int:
 if not SESS_DIR.exists:
 print(f"missing {SESS_DIR}", file=sys.stderr); return 2

 # Default to the canonical N=13 dataset reported in the body. Override
 # with TRIALS env var (comma-separated IDs) or TRIALS=all to render
 # every discovered session for evidence-trail debugging.
 CANONICAL_13 = {
 "C_integrated_nd_hybrid_lidar_depth_08",
 "C_integrated_nd_hybrid_lidar_depth_09",
 "C_integrated_nd_hybrid_lidar_depth_10",
 "C_integrated_nd_only_lidar_depth_01",
 "C_integrated_nd_only_lidar_depth_02",
 "C_integrated_vfh_plus_lidar_depth_01",
 "C_integrated_vfh_plus_lidar_depth_02",
 "C_integrated_nd_hybrid_lidar_only_03",
 "C_integrated_nd_hybrid_lidar_only_04",
 "C_integrated_nd_hybrid_depth_only_01",
 "C_integrated_nd_hybrid_depth_only_02",
 "C_tof_off_safety_nd_hybrid_lidar_depth_03",
 "C_tof_off_safety_nd_hybrid_lidar_depth_04",
 }
 import os
 raw = os.environ.get("TRIALS", "").strip
 if raw == "all":
 allowlist = None
 elif raw:
 allowlist = {t.strip for t in raw.split(",") if t.strip}
 else:
 allowlist = CANONICAL_13

 trials = sorted(p for p in SESS_DIR.iterdir if p.is_symlink)
 panels = []
 for tlink in trials:
 if allowlist is not None and tlink.name not in allowlist:
 continue
 log = tlink / "log.txt"
 if not log.exists:
 continue
 odom, events = parse_session(log)
 if not odom:
 continue
 aligned, theta = align(odom, events=events)
 # Apply any per-trial manual rotation override (operator inspection).
 override_deg = TRIAL_ROTATION_OVERRIDES_DEG.get(tlink.name)
 if override_deg is not None:
 override_rad = math.radians(override_deg)
 cr, sr = math.cos(override_rad), math.sin(override_rad)
 sx, sy = START_X_M, START_Y_M
 aligned = [
 (t,
 (x - sx) * cr - (y - sy) * sr + sx,
 (x - sx) * sr + (y - sy) * cr + sy)
 for t, x, y in aligned
 ]
 panels.append((tlink.name, aligned, events, theta))

 if not panels:
 print("no panels", file=sys.stderr); return 2

 event_styles = {
 "tilt": ("orange", "^"),
 "collision_impact": ("red", "X"),
 "wheel_impact": ("magenta", "*"),
 }

 def make_legend_handles:
 """Proxy handles for the shared figure legend (so the chart is
 self-explanatory without depending on the caption)."""
 from matplotlib.lines import Line2D
 from matplotlib.patches import Patch
 return [
 Line2D([0], [0], marker="o", color="w", label="start",
 markerfacecolor="green", markeredgecolor="black", markersize=9),
 Line2D([0], [0], marker=r"$\rightarrow$", color="green", label="initial heading (+y)",
 markersize=12, linestyle="None"),
 Line2D([0], [0], marker="s", color="w", label="end",
 markerfacecolor="red", markeredgecolor="black", markersize=8),
 Line2D([0], [0], marker="X", color="w", label="dead-end / wardrobe",
 markerfacecolor="black", markeredgecolor="black", markersize=10),
 Line2D([0], [0], marker="^", color="w", label="tilt event",
 markerfacecolor="orange", markeredgecolor="black", markersize=10),
 Line2D([0], [0], marker="X", color="w", label="collision impact",
 markerfacecolor="red", markeredgecolor="black", markersize=10),
 Line2D([0], [0], marker="*", color="w", label="wheel impact",
 markerfacecolor="magenta", markeredgecolor="black", markersize=12),
 Patch(facecolor="#b0c4de", edgecolor="none", label="path (light = early, dark = late)"),
 Patch(facecolor="lightgray", edgecolor="gray", label="obstacle"),
 ]

 # --- Grid view -------------------------------------------------------
 # 3 cols max so individual panels are readable at report scale (a
 # 4-col grid puts each panel below ~ 2 inches in print, where the
 # path traces become illegible).
 n = len(panels)
 cols = 3
 rows = math.ceil(n / cols)
 fig, axes = plt.subplots(rows, cols, figsize=(cols * 5.5, rows * 4.5))
 axes_flat = axes.flatten if rows > 1 else ([axes] if cols == 1 else list(axes))

 # Clamp viewing window to the arena box (plus a little headroom for
 # odometry drift / paths that escaped the physical arena).
 pad = 0.5
 x_lim = (-pad, ARENA_WIDTH_M + pad)
 y_lim = (-pad, ARENA_HEIGHT_M + pad)

 for i, (name, odom, events, theta) in enumerate(panels):
 ax = axes_flat[i]
 xs = [r[1] for r in odom]
 ys = [r[2] for r in odom]
 ts = [r[0] for r in odom]
 # Draw the arena box + dead-end marker + obstacles
 draw_arena(ax, show_obstacles=True)
 # Time-coloured path: LIGHT at start, DARK at end. Dense clusters of
 # dark dots indicate where the robot spent the most time (slow
 # motion, recovery cycles, wedge events).
 ax.scatter(xs, ys, c=ts, s=6, cmap="Blues", vmin=min(ts),
 vmax=max(ts), alpha=0.85, zorder=3)
 ax.plot(xs, ys, color="lightgray", linewidth=0.3, zorder=2)
 # Start (in arena frame)
 ax.scatter(START_X_M, START_Y_M, color="green", s=90, marker="o",
 edgecolors="black", zorder=4)
 ax.scatter(xs[-1], ys[-1], color="red", s=60, marker="s", edgecolors="black", zorder=4)
 # initial heading arrow (vertical, +y)
 ax.annotate("", xy=(START_X_M, START_Y_M + 0.4), xytext=(START_X_M, START_Y_M),
 arrowprops=dict(arrowstyle="->", color="green", lw=1.5))

 ev_count = {"tilt": 0, "collision_impact": 0, "wheel_impact": 0}
 for et, ev in events:
 if ev not in event_styles:
 continue
 ev_count[ev] += 1
 xy = xy_at(et, odom)
 if xy is not None:
 color, marker = event_styles[ev]
 ax.scatter(xy[0], xy[1], color=color, marker=marker, s=140,
 edgecolors="black", linewidths=0.6, zorder=5)

 n_ev = sum(ev_count.values)
 rate = n_ev / 2.5
 ax.set_title(
 f"{name}\ntilt={ev_count['tilt']} imp={ev_count['collision_impact']} "
 f"wimp={ev_count['wheel_impact']} → {rate:.2f}/min",
 fontsize=8,
 )
 ax.set_xlim(*x_lim); ax.set_ylim(*y_lim)
 ax.set_aspect("equal")
 ax.grid(True, alpha=0.3)
 ax.tick_params(labelsize=7)
 ax.axhline(0, color="black", linewidth=0.3, alpha=0.3)
 ax.axvline(0, color="black", linewidth=0.3, alpha=0.3)

 for j in range(n, len(axes_flat)):
 axes_flat[j].axis("off")

 fig.suptitle(
 f"All trial paths — arena frame ({ARENA_WIDTH_M:.1f}×{ARENA_HEIGHT_M:.1f} m, "
 f"start at ({START_X_M:.0f},{START_Y_M:.0f}), heading +y).",
 fontsize=11, y=0.995,
 )
 # Generous vertical space between rows so per-panel titles don't collide
 # with the panel above's x-axis ticks. tight_layout's rect leaves room
 # at the top for the suptitle.
 fig.tight_layout(rect=[0, 0, 1, 0.93], h_pad=2.6, w_pad=1.2)
 fig.legend(handles=make_legend_handles, loc="upper center",
 bbox_to_anchor=(0.5, 0.97), ncol=5, fontsize=14,
 frameon=True, fancybox=True, framealpha=0.95,
 handletextpad=0.5, columnspacing=1.6, markerscale=1.6,
 borderpad=0.8)
 out_grid = ROOT / "report" / "data" / "aligned_paths_grid.png"
 fig.savefig(out_grid, dpi=110)
 print(f"wrote {out_grid} ({n} panels)")

 # --- Overlay view ----------------------------------------------------
 # All trials in one chart, coloured by trial-relative time (LIGHT at
 # start of each trial, DARK at end). Overlap density shows where
 # robots collectively spent the most time. Single colour scheme so the
 # eye reads "dark = time spent here" rather than "trial X = this hue".
 fig2, ax2 = plt.subplots(figsize=(12, 12))
 draw_arena(ax2, show_obstacles=True)
 # Build a single (xs, ys, ts_norm) array across all trials, each trial's
 # time normalised to [0, 1] so all panels share the same colour range.
 all_xs, all_ys, all_t = [], [], []
 for name, odom, events, _ in panels:
 if not odom:
 continue
 t_lo = odom[0][0]
 t_hi = odom[-1][0]
 span = max(t_hi - t_lo, 1e-6)
 xs = [r[1] for r in odom]
 ys = [r[2] for r in odom]
 ts = [(r[0] - t_lo) / span for r in odom]
 all_xs.extend(xs); all_ys.extend(ys); all_t.extend(ts)
 # path line in light grey, so the underlying time-scatter pops
 ax2.plot(xs, ys, color="#888888", linewidth=0.5, alpha=0.25, zorder=2)
 # end-of-trial marker
 ax2.scatter(xs[-1], ys[-1], color="#222222", marker="s", s=40,
 edgecolors="white", linewidths=0.6, alpha=0.7, zorder=4)
 for et, ev in events:
 if ev not in event_styles:
 continue
 xy = xy_at(et, odom)
 if xy is None: continue
 ec, marker = event_styles[ev]
 ax2.scatter(xy[0], xy[1], color=ec, marker=marker, s=130,
 edgecolors="black", linewidths=0.6, alpha=0.9, zorder=5)
 sc = ax2.scatter(all_xs, all_ys, c=all_t, s=6, cmap="Blues",
 vmin=0.0, vmax=1.0, alpha=0.65, zorder=3)
 cbar = fig2.colorbar(sc, ax=ax2, shrink=0.65, pad=0.02)
 cbar.set_label("time within trial (0 = start, 1 = end)", fontsize=8)
 cbar.ax.tick_params(labelsize=7)
 ax2.scatter(START_X_M, START_Y_M, color="green", s=250, marker="o",
 edgecolors="black", zorder=10, label="start (1, 1)")
 ax2.annotate("", xy=(START_X_M, START_Y_M + 0.6), xytext=(START_X_M, START_Y_M),
 arrowprops=dict(arrowstyle="->", color="green", lw=2.2))
 ax2.set_xlabel("x (m, arena frame)")
 ax2.set_ylabel("y (m, arena frame)")
 ax2.set_xlim(-0.5, ARENA_WIDTH_M + 0.5)
 ax2.set_ylim(-0.5, ARENA_HEIGHT_M + 0.5)
 ax2.set_aspect("equal")
 ax2.grid(True, alpha=0.3)
 ax2.set_title(
 f"All trial paths overlaid — arena {ARENA_WIDTH_M:.1f}×{ARENA_HEIGHT_M:.1f} m, "
 f"start ({START_X_M:.0f},{START_Y_M:.0f}) facing +y, dead-end at (0, 0).\n"
 "Light → dark = early → late within each trial. Dark clusters = where the "
 "robot spent the most time.",
 fontsize=10,
 )
 fig2.tight_layout(rect=[0, 0.06, 1, 1])
 fig2.legend(handles=make_legend_handles, loc="upper center",
 bbox_to_anchor=(0.5, 0.20), ncol=5, fontsize=9,
 frameon=True, fancybox=True, framealpha=0.95,
 handletextpad=0.4, columnspacing=1.2)
 out_overlay = ROOT / "report" / "data" / "aligned_paths_overlay.png"
 fig2.savefig(out_overlay, dpi=120)
 print(f"wrote {out_overlay}")

 # --- CSV ------------------------------------------------------------
 out_csv = ROOT / "report" / "data" / "aligned_paths.csv"
 with out_csv.open("w", newline="") as fh:
 w = csv.writer(fh)
 w.writerow(["trial_id", "t_s", "x_aligned", "y_aligned"])
 for name, odom, _, _ in panels:
 for t, x, y in odom:
 w.writerow([name, round(t, 3), round(x, 3), round(y, 3)])
 print(f"wrote {out_csv}")
 return 0


if __name__ == "__main__":
 sys.exit(main)
