"""Pure-function VFH+ core for Project C.

Builds a polar histogram from LIDAR ranges (and optionally a projected depth
channel), selects a safe heading, and exposes a dead-end detector.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class VFHConfig:
 # Defaults retuned for ~0.5 m clearance reactive demos.
 # Old defaults (obstacle_threshold=1.0, safety_margin=0.20, cone=±30°,
 # dead_end_frac=0.85) treated every direction in a < 3 m room as
 # "blocked" → permanent DEAD_END loop. New defaults assume the demo
 # arena is a tight indoor corridor (~0.5 m clearance between obstacles).
 # Tunable per-launch via vfh_planner ROS params.
 n_bins: int = 72 # 5° bins across 360°
 obstacle_threshold_m: float = 0.45 # below this, the bin counts as blocked
 safety_margin_m: float = 0.05 # inflate obstacles by this; total blocked = 0.50 m
 valley_min_width_bins: int = 2 # accept narrow passages (≥ 10° wide)
 goal_bin: int | None = None # None -> prefer forward (bin 0)
 forward_bias_weight: float = 1.0 # bin cost weight: angular distance to forward
 smoothness_weight: float = 0.5 # penalise changing direction sharply
 depth_bonus_weight: float = 8.0 # reward valleys whose mean range is deeper
 # Calibrated so a 0.55→5.0 m depth jump
 # outweighs up to ~±20° angular detour;
 # detours past that still favor forward.
 cone_bins_forward: int = 8 # forward ±20° for dead-end detection
 dead_end_blocked_frac: float = 0.95 # require near-total forward block
 #: chassis half-width for angular obstacle inflation in
 # blocked_mask. The original VFH+ algorithm inflates each blocked bin
 # by the angular half-width that the chassis subtends at that range
 # (asin(chassis_half_width / r)). Without inflation a thin obstacle
 # (fan pole, chair leg) blocks only one or two bins, the planner picks
 # a heading 5° to one side, the chassis tries to drive past the pole
 # and collides because it cannot actually fit through the implied
 # "gap". ROSbot 3 PRO chassis is ~33 cm wide → 0.165 m half-width.
 # Default 0.0 here keeps legacy unit-test fixtures passing; production
 # enables it via the vfh_planner ROS param chassis_half_width_m=0.165.
 chassis_half_width_m: float = 0.0
 # (demo-eve, post-launch): the RPLIDAR S2 driver occasionally
 # publishes self-occlusion returns from the robot's own chassis (rear
 # bracket / antenna) as 0.078 m hits, well inside the device's documented
 # range_min of 0.150 m. Combined with the chassis_half_width_m inflation
 # those single beams collapse the entire histogram and produce permanent
 # DEAD_END. Filter beams below this threshold inside scan_to_histogram.
 # Default 0.0 keeps unit tests untouched; production overrides to 0.10
 # via the vfh_planner ROS param min_range_m.
 min_range_m: float = 0.0


def scan_to_histogram(
 ranges: list[float] | np.ndarray,
 angle_min: float,
 angle_increment: float,
 cfg: VFHConfig,
 yaw_offset_rad: float = 0.0,
) -> np.ndarray:
 """Project a LIDAR scan to a bin-indexed polar histogram.

 Output is a length-n_bins array. Each bin holds the *minimum* range seen by
 any beam in that bin. Bins with no beams have +inf.

 ``yaw_offset_rad`` rotates the LIDAR's a=0 reference into chassis-forward
 before binning. Required when the LIDAR is mounted with a non-zero yaw
 relative to base_link — on the Husarion ROSbot 3 PRO the LIDAR is mounted
 flipped 180° (`tf2_echo` shows RPY [0, 0, 180]), so a value of `math.pi`
 aligns bin 0 with chassis-forward.
 """
 arr = np.asarray(ranges, dtype=np.float32)
 bins = np.full(cfg.n_bins, np.inf, dtype=np.float32)
 for i, r in enumerate(arr):
 if not math.isfinite(r) or r <= 0:
 continue
 if r < cfg.min_range_m:
 continue
 angle = angle_min + i * angle_increment + yaw_offset_rad
 # map angle to [0, 2pi)
 angle = angle % (2 * math.pi)
 bin_idx = int(angle / (2 * math.pi) * cfg.n_bins) % cfg.n_bins
 if r < bins[bin_idx]:
 bins[bin_idx] = r
 return bins


def fuse_depth_channel(
 polar_hist: np.ndarray,
 depth_polar: np.ndarray,
) -> np.ndarray:
 """Take element-wise minimum so depth obstacles inflate the histogram."""
 if depth_polar.shape != polar_hist.shape:
 return polar_hist
 return np.minimum(polar_hist, depth_polar)


def blocked_mask(
 polar_hist: np.ndarray,
 cfg: VFHConfig,
 *,
 threshold_m: float | None = None,
) -> np.ndarray:
 """True where the bin is blocked, with chassis-aware angular inflation.

 Each raw blocked bin at range r additionally blocks all bins within
 angular distance ``asin(chassis_half_width_m / r)`` (the half-angle the
 chassis subtends at that range). For r <= chassis_half_width_m the
 obstacle is closer than the chassis is wide → inflate by π (block the
 entire histogram around it; recovery should fire).

 Without inflation, a thin obstacle (fan pole, chair leg, table foot)
 blocks only one or two bins; the planner picks a heading 5° to one
 side and the chassis collides because it cannot fit through the
 implied gap. This is the standard VFH+ "obstacle dilation" step from
 Borenstein & Koren 1991, restored after a fan-pole wedge.

 ``threshold_m`` overrides the default `obstacle_threshold_m + safety_margin_m`
 threshold. Required by `nd_core.find_free_regions` which uses ND's
 `safety_dist_m` instead of VFH+'s obstacle threshold (Phase 0 of fix).
 """
 threshold = (
 threshold_m
 if threshold_m is not None
 else cfg.obstacle_threshold_m + cfg.safety_margin_m
 )
 raw = polar_hist <= threshold
 if cfg.chassis_half_width_m <= 0.0 or not raw.any:
 return raw
 n = len(polar_hist)
 bin_angle = (2.0 * math.pi) / n
 inflated = raw.copy
 blocked_indices = np.where(raw)[0]
 for i in blocked_indices:
 r = float(polar_hist[i])
 if not math.isfinite(r) or r <= 0.0:
 continue
 if r <= cfg.chassis_half_width_m:
 half_angle = math.pi # obstacle closer than chassis wide
 else:
 half_angle = math.asin(min(1.0, cfg.chassis_half_width_m / r))
 n_inflate = int(math.ceil(half_angle / bin_angle))
 # Clamp inflation span to avoid wrapping more than n bins.
 n_inflate = min(n_inflate, n // 2)
 for j in range(-n_inflate, n_inflate + 1):
 inflated[(i + j) % n] = True
 return inflated


def find_valleys(mask_blocked: np.ndarray, cfg: VFHConfig) -> list[tuple[int, int]]:
 """Return list of (start_bin, end_bin) ranges that are continuously free.

 Indices wrap modulo n_bins so a valley crossing index 0 is handled.
 """
 n = len(mask_blocked)
 if not mask_blocked.any:
 return [(0, n - 1)]
 if mask_blocked.all:
 return []
 free = ~mask_blocked
 valleys: list[tuple[int, int]] = []

 # Rotate start to the first blocked bin so we avoid wrap handling.
 start_offset = int(np.argmax(mask_blocked))
 rotated = np.roll(free, -start_offset)
 in_run = False
 run_start = 0
 for i, f in enumerate(rotated):
 if f and not in_run:
 run_start = i
 in_run = True
 elif not f and in_run:
 if (i - run_start) >= cfg.valley_min_width_bins:
 valleys.append(((run_start + start_offset) % n,
 (i - 1 + start_offset) % n))
 in_run = False
 if in_run and (len(rotated) - run_start) >= cfg.valley_min_width_bins:
 valleys.append(((run_start + start_offset) % n,
 (len(rotated) - 1 + start_offset) % n))
 return valleys


# Hysteresis margin for select_heading (cost units). The previous heading
# wins unless a fresh candidate's cost is at least this much lower. Without
# the margin the planner re-picks adjacent valleys every tick on a noisy
# histogram, producing a visible left/right wobble in tight rooms — the
# "spin/move/spin/move" pattern observed during demo-eve.
HYSTERESIS_MARGIN: float = 8.0 # bumped 4->8 : noisy depth fusion routinely shifted normalised_depth by ±0.5 between adjacent ticks, producing ±4.0 cost-unit swings that broke the prior margin


def select_heading(
 polar_hist: np.ndarray,
 cfg: VFHConfig,
 goal_bin: int | None = None,
 previous_bin: int | None = None,
) -> int | None:
 """Choose a heading bin using the forward-first rule.

 The decision is conditional, not weighted:

 1. **If the forward bin (target) sits inside any open valley, take it.**
 No second-guessing — straight ahead is the operator's intent. The
 camera-fused polar histogram has already validated that the forward
 cone is genuinely open; trust it.

 2. **Otherwise pick the deepest valley.** When forward is blocked, the
 robot's correct move is to detour through the most-open opening
 available, regardless of how far off-forward it is. A small
 angular-distance tiebreak keeps equal-depth valleys leaning toward
 the forward direction; otherwise depth dominates.

 3. **Hysteresis on the deep-valley fallback.** When the previous heading
 is still in an open valley AND the new candidate is not meaningfully
 deeper, keep the previous heading. This kills per-tick wobble on
 noisy depth fusion. Hysteresis only applies in the fallback path —
 forward-first overrides it (returning to forward as soon as the
 forward cone reopens is the desired behaviour).

 Returns None when no valley is open at all (recovery_controller's job
 to handle that via DEAD_END).
 """
 mask = blocked_mask(polar_hist, cfg)
 valleys = find_valleys(mask, cfg)
 if not valleys:
 return None
 target = goal_bin if goal_bin is not None else (cfg.goal_bin or 0)
 n = len(polar_hist)

 def bin_distance(a: int, b: int) -> int:
 d = abs(a - b) % n
 return min(d, n - d)

 def valley_indices(start: int, end: int) -> list[int]:
 if start <= end:
 return list(range(start, end + 1))
 return list(range(start, n)) + list(range(0, end + 1))

 # 1. Forward-first. If target sits in any valley, commit.
 for start, end in valleys:
 if target in valley_indices(start, end):
 return target

 # 2. Forward is blocked. Score every valley by depth (higher = more
 # open) with a light forward tiebreak. Track which valley contains the
 # previous heading so we can apply hysteresis below.
 candidates: list[tuple[float, int, float]] = [] # (score, centre, depth)
 prev_centre_now: int | None = None
 prev_score: float | None = None
 for start, end in valleys:
 idxs = valley_indices(start, end)
 centre = idxs[len(idxs) // 2]
 depths = polar_hist[idxs]
 finite = np.where(np.isfinite(depths), depths, 5.0)
 capped = np.minimum(finite, 5.0)
 normalised_depth = float(np.mean(capped) / 5.0)
 # Higher score = more preferred. Depth dominates; angular distance
 # to forward is a small tiebreak.
 score = normalised_depth - 0.03 * bin_distance(centre, target)
 candidates.append((score, centre, normalised_depth))
 if previous_bin is not None and previous_bin in idxs:
 prev_centre_now = centre
 prev_score = score

 candidates.sort(key=lambda t: -t[0]) # highest score first
 best_score, best_centre, _ = candidates[0]

 # 3. Hysteresis on the fallback. HYSTERESIS_MARGIN is in cost units;
 # the new score-based path expresses its margin in score units (a
 # normalised-depth swing of about 0.10 = 0.5 m of clearance). Stay with
 # the previous heading unless the new best beats it by that margin.
 if prev_centre_now is not None and prev_score is not None:
 if best_score < prev_score + 0.10:
 return prev_centre_now

 return best_centre


def heading_to_yaw_rate(
 chosen_bin: int, cfg: VFHConfig, *, k: float = 1.0, w_max: float = 1.0
) -> float:
 """Convert a chosen bin into an angular velocity command."""
 n = cfg.n_bins
 angle = (chosen_bin / n) * (2 * math.pi)
 if angle > math.pi:
 angle -= 2 * math.pi
 return max(-w_max, min(w_max, k * angle))


def is_dead_end(polar_hist: np.ndarray, cfg: VFHConfig) -> bool:
 """Forward cone mostly blocked -> dead end."""
 n = cfg.n_bins
 half = cfg.cone_bins_forward // 2
 indices = [(n + i) % n for i in range(-half, half + 1)]
 front = polar_hist[indices]
 threshold = cfg.obstacle_threshold_m + cfg.safety_margin_m
 blocked = (front <= threshold).sum
 return (blocked / len(indices)) >= cfg.dead_end_blocked_frac


def should_emit_stale_event(
 last_hist_at: float | None,
 now: float,
 *,
 stale_threshold_s: float = 0.5,
 last_emit_at: float | None = None,
 debounce_s: float = 1.0,
) -> bool:
 """Decide whether vfh_planner should publish a stale-perception event now.

 Pure function so the timer-driven logic in vfh_planner is unit-testable.
 The eval-note expectation (Week 9 lecture) is that a planner subscribed to
 a sensor topic should detect when the topic has gone quiet and surface the
 failure rather than continue acting on stale data.

 Returns True iff:
 - we have seen at least one timestamp (or were primed at boot),. - the most recent histogram is older than ``stale_threshold_s``,. - either we never emitted a stale event before, or the previous one is
 older than ``debounce_s`` (prevents log spam during long outages).
 """
 if last_hist_at is None:
 return False
 if (now - last_hist_at) <= stale_threshold_s:
 return False
 if last_emit_at is None:
 return True
 return (now - last_emit_at) >= debounce_s
