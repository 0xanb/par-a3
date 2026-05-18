"""Nearness Diagram (ND) reactive-navigation core.

Pure-function classifier + per-state controllers. No ROS imports — exercised
by unit tests against synthetic polar histograms.

Algorithm: J. Minguez and L. Montano, "Nearness Diagram (ND) Navigation:
Collision Avoidance in Troublesome Scenarios," IEEE T-RA vol. 20 no. 1,
Feb. 2004, pp. 45-59. doi:10.1109/TRA.2003.820849.

Five canonical situations (terms from the paper):
 HSGR High Safety, Goal in Region — open path toward goal, cruise.
 HSWR High Safety, Wide Region — goal occluded, drive widest gap.
 HSNR High Safety, Narrow Region — narrow gap, careful threading.
 LS1 Low Safety, one obstacle close — drift away while progressing.
 LS2 Low Safety, both sides close — drive bisector of the two.

A sixth runtime case is added: DEAD_END_LS2 — low safety AND no escape region
exists at all (geometric dead-end). The robot reverses + spins until the
classifier finds a region above the safety threshold. (Renamed from "LS2_BACKUP"
in the hybrid so the recovery_controller can pattern-match on
labels starting with "DEAD_END" —md.)

This implementation deliberately uses the same `polar_hist` topic and the
same arbiter publication contract (`source="reactive"`, priority 70) as the
VFH+ planner, so the two planners are drop-in alternatives behind their own
launch files (project_c.launch.py = ND, project_c0.launch.py = VFH+).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from.vfh_core import VFHConfig, blocked_mask


@dataclass
class NDConfig:
 """Tunable parameters for the ND classifier and per-state controllers."""
 n_bins: int = 72 # 5° per bin across 360°
 R_max_m: float = 5.0 # sensor saturation range
 safety_dist_m: float = 0.30 # chassis_half_width + margin + headroom
 wide_threshold_bins: int = 10 # ≥ 50° = "wide" free region
 valley_min_width_bins: int = 2 # discard regions narrower than this
 goal_bin: int = 0 # bin 0 = forward
 cruise_v: float = 0.12 # nominal forward speed (m/s); lowered for gentler-impact demo arena
 cruise_w_max: float = 1.0 # max yaw rate (rad/s)
 forward_cone_bins: int = 24 # ±60° "active" cone for safety check
 backup_v: float = -0.10 # DEAD_END_LS2 linear speed
 backup_w: float = 0.8 # DEAD_END_LS2 angular speed (turn while reversing)
 # Chassis half-width used for the same Borenstein-Koren angular dilation
 # as VFH+. Default 0.0 keeps unit-test fixtures unchanged; production
 # overrides via the nd_planner ROS param chassis_half_width_m=0.165.
 chassis_half_width_m: float = 0.0


@dataclass
class FreeRegion:
 """A contiguous run of bins whose range exceeds `safety_dist_m`."""
 start: int # inclusive
 end: int # inclusive
 width: int # number of bins (end - start + 1, modulo wrap)
 center: int # central bin
 min_range: float # min hist value across the region (always > safety_dist)


def _vfh_cfg_for_inflation(cfg: NDConfig) -> VFHConfig:
 """Build a VFHConfig that lets `vfh_core.blocked_mask` apply ND's
 chassis-aware angular inflation without imposing VFH+'s thresholds.

 Only the fields blocked_mask reads matter (n_bins, chassis_half_width_m).
 obstacle_threshold_m / safety_margin_m are forced to 0 because we always
 pass `threshold_m=cfg.safety_dist_m` explicitly to override the default.
 """
 return VFHConfig(
 n_bins=cfg.n_bins,
 chassis_half_width_m=cfg.chassis_half_width_m,
 obstacle_threshold_m=0.0,
 safety_margin_m=0.0,
 )


def find_free_regions(
 polar_hist: np.ndarray,
 cfg: NDConfig,
) -> list[FreeRegion]:
 """Identify contiguous bin runs whose range exceeds `safety_dist_m`.

 Wraps around bin n-1 → bin 0 (the histogram is circular). A region that
 spans the wrap is reported as a single FreeRegion with start > end (the
 caller's `width` field handles the wrap correctly).

 When `cfg.chassis_half_width_m > 0`, blocked bins are first inflated by
 Borenstein-Koren angular dilation (each blocked bin at range r additionally
 blocks ±asin(chassis_half_width / r) of arc) before regions are computed
 from the inverted mask. This prevents the wedge where a thin obstacle
 blocks one bin, the bins immediately adjacent are still "free", and the
 planner picks a heading where the chassis cannot fit. Pre-behaviour
 (raw threshold, no inflation) is preserved when chassis_half_width_m == 0.
 """
 n = cfg.n_bins
 if cfg.chassis_half_width_m > 0.0:
 mask = blocked_mask(
 polar_hist,
 _vfh_cfg_for_inflation(cfg),
 threshold_m=cfg.safety_dist_m,
 )
 free = ~mask
 else:
 free = polar_hist >= cfg.safety_dist_m
 if not free.any:
 return []
 if free.all:
 return [FreeRegion(0, n - 1, n, n // 2, float(np.min(polar_hist)))]

 # Find run boundaries on the circular array. We rotate to start at the
 # first non-free bin so transitions are well-defined.
 first_blocked = int(np.argmin(free))
 rotated = np.roll(free, -first_blocked)
 regions: list[FreeRegion] = []
 i = 0
 while i < n:
 if not rotated[i]:
 i += 1
 continue
 j = i
 while j < n and rotated[j]:
 j += 1
 # Run [i, j-1] in the rotated frame.
 width = j - i
 if width >= cfg.valley_min_width_bins:
 start = (i + first_blocked) % n
 end = (j - 1 + first_blocked) % n
 center = (i + j - 1) // 2
 center = (center + first_blocked) % n
 min_range = float(np.min(polar_hist[start:end + 1] if start <= end
 else np.concatenate([polar_hist[start:], polar_hist[:end + 1]])))
 regions.append(FreeRegion(start, end, width, center, min_range))
 i = j
 return regions


def region_contains(region: FreeRegion, bin_idx: int, n_bins: int) -> bool:
 """Does the (possibly wrap-spanning) region include bin_idx?"""
 if region.start <= region.end:
 return region.start <= bin_idx <= region.end
 # Wrap region: [start, n-1] ∪ [0, end]
 return bin_idx >= region.start or bin_idx <= region.end


def nearest_obstacle_side(
 polar_hist: np.ndarray,
 cfg: NDConfig,
) -> tuple[float, float]:
 """Return (left_min_range, right_min_range) within the forward cone.

 Left = bins +1 to +cone/2 (inclusive). Right = bins -cone/2 to -1.
 Used to discriminate LS1 (one side close) from LS2 (both sides close).
 """
 n = cfg.n_bins
 half = cfg.forward_cone_bins // 2
 left_idxs = [(cfg.goal_bin + i) % n for i in range(1, half + 1)]
 right_idxs = [(cfg.goal_bin - i) % n for i in range(1, half + 1)]
 left_min = float(min(polar_hist[i] for i in left_idxs))
 right_min = float(min(polar_hist[i] for i in right_idxs))
 return left_min, right_min


@dataclass
class NDDecision:
 """Output of one classify+control tick."""
 label: str # one of HSGR, HSWR, HSNR, LS1, LS2, DEAD_END_LS2 (post rename), DEAD_END_WEDGE (planner-level watchdog)
 v: float # linear velocity command (m/s)
 w: float # angular velocity command (rad/s)
 chosen_bin: int | None # selected heading bin (None for DEAD_END_LS2)
 confidence: float # 0.0 to 1.0


def _bin_to_yaw(chosen_bin: int, cfg: NDConfig) -> float:
 """Convert a histogram bin index to a yaw target (radians)."""
 n = cfg.n_bins
 # Bins 0n/2 are positive-yaw (left-handed convention used by /scan).
 # Bin 0 = forward (yaw = 0). Bin n/2 = behind (yaw = ±π).
 bin_signed = chosen_bin if chosen_bin <= n // 2 else chosen_bin - n
 return (2.0 * math.pi / n) * bin_signed


def _yaw_to_w(yaw_target: float, cfg: NDConfig, k: float = 1.5) -> float:
 """Proportional controller: yaw error -> yaw rate, clamped to ±cruise_w_max."""
 w = k * yaw_target
 return max(-cfg.cruise_w_max, min(cfg.cruise_w_max, w))


def classify_and_command(
 polar_hist: np.ndarray,
 cfg: NDConfig,
) -> NDDecision:
 """Single-tick classifier + per-state controller.

 The classifier examines the polar histogram and selects one of six
 situations (5 paper states + DEAD_END_LS2), then dispatches to the
 appropriate per-state controller for the (v, w) command.
 """
 n = cfg.n_bins
 half_cone = cfg.forward_cone_bins // 2
 # Active cone = forward ±half_cone bins (wraps).
 cone_idxs = [(cfg.goal_bin + i) % n for i in range(-half_cone, half_cone + 1)]
 cone_min = float(min(polar_hist[i] for i in cone_idxs))
 high_safety = cone_min >= cfg.safety_dist_m

 regions = find_free_regions(polar_hist, cfg)

 # --- DEAD_END_LS2: low safety AND no usable free region anywhere ---
 if not high_safety and not regions:
 return NDDecision(
 label="DEAD_END_LS2",
 v=cfg.backup_v,
 w=cfg.backup_w,
 chosen_bin=None,
 confidence=1.0,
 )

 # --- High Safety branch ---
 if high_safety:
 if not regions:
 # Pathological: high safety yet no region detected. Fall through
 # to a defensive forward-stop.
 return NDDecision(
 label="HSGR", v=0.0, w=0.0, chosen_bin=cfg.goal_bin, confidence=0.5
 )
 # HSGR: a free region contains the goal (forward) bin.
 for r in regions:
 if region_contains(r, cfg.goal_bin, n):
 yaw = _bin_to_yaw(cfg.goal_bin, cfg)
 w = _yaw_to_w(yaw, cfg)
 return NDDecision(
 label="HSGR",
 v=cfg.cruise_v,
 w=w,
 chosen_bin=cfg.goal_bin,
 confidence=1.0,
 )
 # Goal not in any free region — pick the widest.
 widest = max(regions, key=lambda r: r.width)
 yaw = _bin_to_yaw(widest.center, cfg)
 w = _yaw_to_w(yaw, cfg)
 if widest.width >= cfg.wide_threshold_bins:
 # HSWR: drive into the centre of the wide region at moderate speed.
 return NDDecision(
 label="HSWR",
 v=cfg.cruise_v * 0.8,
 w=w,
 chosen_bin=widest.center,
 confidence=0.9,
 )
 # HSNR: narrow but high safety. Slow + bisector heading.
 return NDDecision(
 label="HSNR",
 v=cfg.cruise_v * 0.5,
 w=w,
 chosen_bin=widest.center,
 confidence=0.8,
 )

 # --- Low Safety branch ---
 left_min, right_min = nearest_obstacle_side(polar_hist, cfg)
 left_close = left_min < cfg.safety_dist_m
 right_close = right_min < cfg.safety_dist_m

 if left_close and right_close:
 # LS2: both sides close — drive bisector of the widest region (the
 # least bad opening) at the lowest speed.
 if regions:
 best = max(regions, key=lambda r: r.width)
 yaw = _bin_to_yaw(best.center, cfg)
 w = _yaw_to_w(yaw, cfg)
 return NDDecision(
 label="LS2",
 v=cfg.cruise_v * 0.3,
 w=w,
 chosen_bin=best.center,
 confidence=0.7,
 )
 # No region at all + low safety = DEAD_END_LS2 (handled above) but
 # defensive fallback if reached.
 return NDDecision(
 label="DEAD_END_LS2",
 v=cfg.backup_v,
 w=cfg.backup_w,
 chosen_bin=None,
 confidence=0.7,
 )

 # LS1: one side close. Drift away from it while still moving forward.
 # Bias the heading by ~half the cone width away from the close side.
 bias_bins = cfg.forward_cone_bins // 4 # quarter-cone away
 if left_close:
 target_bin = (cfg.goal_bin - bias_bins) % n # turn right
 else:
 target_bin = (cfg.goal_bin + bias_bins) % n # turn left
 yaw = _bin_to_yaw(target_bin, cfg)
 w = _yaw_to_w(yaw, cfg)
 return NDDecision(
 label="LS1",
 v=cfg.cruise_v * 0.4,
 w=w,
 chosen_bin=target_bin,
 confidence=0.7,
 )
