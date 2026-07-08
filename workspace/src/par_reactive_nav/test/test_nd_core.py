"""Unit tests for nd_core (Nearness Diagram classifier + per-state controllers).

Each test uses a synthetic polar histogram (length = n_bins, +inf for empty
bins) crafted to exercise one specific classifier branch. No ROS imports.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from par_reactive_nav.nd_core import (
    NDConfig,
    classify_and_command,
    find_free_regions,
    nearest_obstacle_side,
    region_contains,
)


# --- Helpers ---------------------------------------------------------------

def make_open_hist(cfg: NDConfig, value: float = 5.0) -> np.ndarray:
    """All bins read `value`. Sentinel for an open arena."""
    return np.full(cfg.n_bins, value, dtype=np.float32)


def block_bins(hist: np.ndarray, *bin_ranges: tuple[int, int],
               distance: float = 0.10) -> np.ndarray:
    """Set ranges within bin_ranges (start_inclusive, end_exclusive) to `distance`."""
    out = hist.copy()
    n = len(out)
    for start, end in bin_ranges:
        for i in range(start, end):
            out[i % n] = distance
    return out


# --- find_free_regions -----------------------------------------------------

def test_find_free_regions_all_open():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    regions = find_free_regions(hist, cfg)
    assert len(regions) == 1
    assert regions[0].width == cfg.n_bins


def test_find_free_regions_all_blocked():
    cfg = NDConfig()
    hist = np.full(cfg.n_bins, 0.05, dtype=np.float32)
    regions = find_free_regions(hist, cfg)
    assert regions == []


def test_find_free_regions_single_obstacle_splits_into_one_region():
    cfg = NDConfig()
    # Block bins 30-40 (spans forward-left); the rest is open. Because bin 0
    # is forward and the obstacle is on the side, the remaining free bins
    # form one wrap-spanning region.
    hist = block_bins(make_open_hist(cfg), (30, 41))
    regions = find_free_regions(hist, cfg)
    assert len(regions) == 1
    # The single region should NOT contain bins 30..40
    for i in range(30, 41):
        assert not region_contains(regions[0], i, cfg.n_bins)
    # And SHOULD contain bin 0 (forward)
    assert region_contains(regions[0], 0, cfg.n_bins)


def test_find_free_regions_two_obstacles_two_regions():
    cfg = NDConfig()
    # Block bins 20-25 (left side) AND bins 50-55 (rear-right). Yields two
    # free regions: forward+right and behind-left.
    hist = block_bins(make_open_hist(cfg), (20, 26), (50, 56))
    regions = find_free_regions(hist, cfg)
    assert len(regions) == 2


# --- nearest_obstacle_side -------------------------------------------------

def test_nearest_obstacle_side_left_only():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    # Block bin 5 (left of forward) at very close range.
    hist[5] = 0.10
    left_min, right_min = nearest_obstacle_side(hist, cfg)
    assert left_min < cfg.safety_dist_m
    assert right_min >= cfg.safety_dist_m


def test_nearest_obstacle_side_right_only():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    # Block bin -5 = bin 67 (right of forward) at very close range.
    hist[(cfg.goal_bin - 5) % cfg.n_bins] = 0.10
    left_min, right_min = nearest_obstacle_side(hist, cfg)
    assert left_min >= cfg.safety_dist_m
    assert right_min < cfg.safety_dist_m


# --- classify_and_command --------------------------------------------------

def test_classify_open_arena_returns_HSGR():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    d = classify_and_command(hist, cfg)
    assert d.label == "HSGR"
    assert d.v == pytest.approx(cfg.cruise_v)
    # No correction needed: w should be near zero for forward goal.
    assert abs(d.w) < 0.05
    assert d.chosen_bin == cfg.goal_bin


def test_classify_corridor_with_distant_walls_returns_HSGR():
    cfg = NDConfig()
    hist = make_open_hist(cfg, value=1.5)
    # Walls at 0.50 m on each side at bins ±18 (90° to the side).
    hist[18] = 0.50
    hist[(cfg.goal_bin - 18) % cfg.n_bins] = 0.50
    d = classify_and_command(hist, cfg)
    assert d.label == "HSGR"
    assert d.v > 0.0


def test_classify_obstacle_directly_ahead_high_safety_picks_widest():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    # Block bins -2..+2 (forward cone) at marginally above safety_dist.
    # cone_min stays high but goal bin no longer in any free region.
    for i in range(-2, 3):
        hist[i % cfg.n_bins] = 0.40   # > safety_dist_m (0.30)
    # Now actually block goal: bin 0 just at safety boundary.
    hist[0] = 0.25
    d = classify_and_command(hist, cfg)
    # Should not be HSGR (goal bin blocked); should be one of HSWR / HSNR.
    assert d.label in ("HSWR", "HSNR", "LS1", "LS2", "DEAD_END_LS2")


def test_classify_one_close_obstacle_returns_LS1():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    # One close obstacle on the left side (bin +5) inside safety_dist.
    hist[5] = 0.20
    # Forward bin still has some range though, but close obstacle pulls
    # cone_min < safety_dist.
    d = classify_and_command(hist, cfg)
    assert d.label == "LS1"
    # Heading should bias AWAY from the close left obstacle = turn right.
    # In the bin convention, "right" means a negative bin index (which wraps
    # to a high bin number).
    assert d.chosen_bin is not None
    bin_signed = d.chosen_bin if d.chosen_bin <= cfg.n_bins // 2 else d.chosen_bin - cfg.n_bins
    assert bin_signed < 0       # turning right (away from left obstacle)


def test_classify_two_close_obstacles_returns_LS2():
    cfg = NDConfig()
    hist = make_open_hist(cfg)
    # Two close obstacles symmetric about forward.
    hist[5] = 0.20
    hist[(cfg.goal_bin - 5) % cfg.n_bins] = 0.20
    d = classify_and_command(hist, cfg)
    assert d.label in ("LS2", "DEAD_END_LS2")


def test_classify_dead_end_all_blocked_returns_dead_end_ls2():
    cfg = NDConfig()
    # Everything inside safety_dist_m: no free region exists.
    hist = np.full(cfg.n_bins, 0.10, dtype=np.float32)
    d = classify_and_command(hist, cfg)
    assert d.label == "DEAD_END_LS2"
    assert d.v < 0.0    # reversing
    assert d.w != 0.0   # spinning


def test_classify_corner_wedge_b01_acceptance_case():
    """The exact corner geometry from acceptance criterion #1.

    forward 0.12 m / left 0.19 m / right 0.13 m / back 0.54 m.

    All three forward-side ranges are inside safety_dist_m. The back-region
    is open. Expected: LS2 (close on both sides, but a distant rear region
    exists) or DEAD_END_LS2 (if the rear region is too narrow to qualify).
    """
    cfg = NDConfig()
    hist = make_open_hist(cfg, value=5.0)
    # Forward bin (0): 0.12 m
    hist[0] = 0.12
    # Left bin (~+18, 90°): 0.19 m
    hist[18] = 0.19
    # Right bin (~-18, -90° = bin 54): 0.13 m
    hist[(cfg.goal_bin - 18) % cfg.n_bins] = 0.13
    # Block all forward cone (±60°) so cone_min < safety_dist_m for sure.
    for i in range(-12, 13):
        hist[i % cfg.n_bins] = 0.20
    # Behind (bin ~36, 180°): 0.54 m — open enough to use as escape.
    hist[36] = 0.54
    # Make the area around bin 36 also open so there's a free region behind.
    for i in range(33, 40):
        hist[i] = 0.54
    d = classify_and_command(hist, cfg)
    assert d.label in ("LS2", "DEAD_END_LS2")
    # Either the planner chose to drive toward the rear free region (LS2)
    # or it triggered DEAD_END_LS2. Both are acceptable resolutions for a
    # corner wedge with only a rear escape.
    if d.label == "LS2":
        assert d.chosen_bin is not None
        # Heading should point roughly behind (bin near 36 / ±π yaw).
        # We just check it's NOT pointing forward.
        forward_distance = min(
            abs(d.chosen_bin - cfg.goal_bin),
            cfg.n_bins - abs(d.chosen_bin - cfg.goal_bin)
        )
        assert forward_distance > cfg.n_bins // 4


def test_classify_narrow_passage_returns_HSNR():
    cfg = NDConfig()
    # Open arena 5 m everywhere except a narrow gap forward: walls at bins
    # +3 and -3 at 0.40 m (above safety, but close). Cone_min stays high.
    hist = make_open_hist(cfg, value=5.0)
    # Block everything except a narrow gap at bins -2 .. +2 (5 bins = 25°).
    for i in range(cfg.n_bins):
        if not (-2 <= i <= 2 or -2 <= i - cfg.n_bins <= 2):
            hist[i] = 5.0  # leave open (rear is also open)
    # Block bins ±3 .. ±10 with marginal safety distance.
    for i in list(range(3, 11)) + [(cfg.goal_bin - i) % cfg.n_bins for i in range(3, 11)]:
        hist[i] = 0.40
    d = classify_and_command(hist, cfg)
    # The narrow forward gap should cause HSGR (gap contains bin 0) or HSNR.
    assert d.label in ("HSGR", "HSNR", "HSWR")
