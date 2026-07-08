"""Regression tests for ND chassis-aware angular obstacle inflation (post ).

Pre-, `nd_core.find_free_regions` derived free regions from a raw
``polar_hist >= safety_dist_m`` threshold and ignored ``chassis_half_width_m``
even though the field was declared in NDConfig and threaded through the ROS
param. Result: a thin obstacle blocks one bin, the bins immediately adjacent
remain "free", and the planner picks a heading where the chassis cannot fit.

Post-, when ``chassis_half_width_m > 0``, the histogram is pre-processed
through `vfh_core.blocked_mask` with ``threshold_m=cfg.safety_dist_m`` so each
blocked bin at range r additionally blocks ±asin(chassis_half_width / r) of
arc. The Borenstein-Koren angular dilation (1991) prevents the wedge.

These tests pin the contract:
  (a) Empty histogram → no inflation.
  (b) Single thin obstacle at r=0.20 m, chassis_w=0.165 → ±55° inflated bins.
  (c) chassis_half_width_m=0 → no inflation (legacy parity).
"""
from __future__ import annotations

import math

import numpy as np

from par_reactive_nav.nd_core import NDConfig, find_free_regions, region_contains


def _open_hist(cfg: NDConfig, value: float = 5.0) -> np.ndarray:
    return np.full(cfg.n_bins, value, dtype=np.float32)


def test_no_inflation_when_chassis_half_width_zero():
    """Legacy parity: chassis_half_width_m=0 disables Borenstein-Koren inflation."""
    cfg = NDConfig(chassis_half_width_m=0.0)
    hist = _open_hist(cfg)
    # Single bin blocked at r=0.20 m. Pre-inflation, only bin 5 is blocked.
    hist[5] = 0.20
    regions = find_free_regions(hist, cfg)
    # Without inflation, bins 4 and 6 remain free → the single free region
    # spans the whole circle except for bin 5.
    assert len(regions) == 1
    assert region_contains(regions[0], 4, cfg.n_bins)
    assert region_contains(regions[0], 6, cfg.n_bins)
    assert not region_contains(regions[0], 5, cfg.n_bins)


def test_inflation_blocks_adjacent_bins_for_thin_obstacle_at_close_range():
    """The headline fix: thin obstacle at 0.20 m with 0.165 chassis half-width
    inflates by asin(0.165/0.20) ≈ 0.978 rad ≈ 56°. With 5°/bin, that's ~11 bins
    per side → bins 5-11 (right) and 5-(-11) wrap (= bins 5..16 forward, plus
    backward dilation onto the wrap). The bins immediately adjacent to the
    obstacle (bins 4, 6) MUST be blocked post-fix.
    """
    cfg = NDConfig(chassis_half_width_m=0.165)
    hist = _open_hist(cfg)
    hist[5] = 0.20
    regions = find_free_regions(hist, cfg)
    # The bin at the obstacle and its immediate neighbours must NOT be in any
    # free region after inflation.
    # find_free_regions returns regions over the inflated mask, so the bins
    # within the inflation half-angle are excluded.
    half_angle_rad = math.asin(cfg.chassis_half_width_m / 0.20)
    bin_angle = 2.0 * math.pi / cfg.n_bins
    n_inflate = int(math.ceil(half_angle_rad / bin_angle))
    assert n_inflate >= 10  # sanity: ~11 bins per side
    for offset in range(-n_inflate, n_inflate + 1):
        target = (5 + offset) % cfg.n_bins
        in_any_region = any(
            region_contains(r, target, cfg.n_bins) for r in regions
        )
        assert not in_any_region, (
            f"Bin {target} (offset {offset:+d} from obstacle at bin 5) "
            f"is still in a free region — chassis inflation did not block it"
        )


def test_inflation_proportional_to_range():
    """An obstacle far away inflates a smaller arc than a close one. r=2.0 m
    with chassis_w=0.165 → asin(0.165/2.0) ≈ 0.083 rad ≈ 4.7° → 1 bin per side
    of inflation. So bins ±2 from the obstacle should still be free.
    """
    cfg = NDConfig(chassis_half_width_m=0.165)
    hist = _open_hist(cfg)
    hist[10] = 2.0   # far-ish thin obstacle
    regions = find_free_regions(hist, cfg)
    # Bins 12 and 8 (±2 from obstacle at bin 10) should be free at r=2.0 m.
    assert any(region_contains(r, 12, cfg.n_bins) for r in regions), (
        "Bin 12 should be free with a far obstacle at bin 10 (small inflation)"
    )
    assert any(region_contains(r, 8, cfg.n_bins) for r in regions), (
        "Bin 8 should be free with a far obstacle at bin 10 (small inflation)"
    )


def test_open_arena_unaffected_by_inflation():
    """An empty histogram has no blocked bins to inflate; the result must be
    a single free region spanning the whole circle, identical to the
    chassis-zero case."""
    cfg = NDConfig(chassis_half_width_m=0.165)
    hist = _open_hist(cfg)
    regions = find_free_regions(hist, cfg)
    assert len(regions) == 1
    assert regions[0].width == cfg.n_bins


def test_obstacle_inside_chassis_envelope_inflates_full_circle():
    """When an obstacle's range is less than the chassis half-width, the
    Borenstein-Koren formula gives half_angle = π → block the entire circle.
    No free regions should remain. This catches the case where a sensor is
    reporting an impossible self-occlusion that should block all motion."""
    cfg = NDConfig(chassis_half_width_m=0.165)
    hist = _open_hist(cfg)
    # Obstacle reported at 0.05 m — impossibly close, inside chassis envelope.
    hist[0] = 0.05
    regions = find_free_regions(hist, cfg)
    # With full-circle inflation, no free regions.
    assert regions == []
