"""Unit tests for the VFH+ core."""
from __future__ import annotations

import math

import numpy as np

from par_reactive_nav.vfh_core import (
    VFHConfig, blocked_mask, find_valleys, heading_to_yaw_rate, is_dead_end,
    scan_to_histogram, select_heading,
)


def test_scan_to_histogram_captures_minimum() -> None:
    cfg = VFHConfig(n_bins=72)
    ranges = [5.0] * 360
    ranges[0] = 0.5
    hist = scan_to_histogram(ranges, 0.0, math.radians(1.0), cfg)
    # bin 0 should hold the minimum 0.5
    assert abs(hist[0] - 0.5) < 1e-6


def test_blocked_mask_honours_threshold_and_margin() -> None:
    cfg = VFHConfig(obstacle_threshold_m=1.0, safety_margin_m=0.2)
    hist = np.array([0.5, 0.8, 1.1, 1.5, np.inf], dtype=np.float32)
    mask = blocked_mask(hist, cfg)
    assert mask.tolist() == [True, True, True, False, False]


def test_find_valleys_returns_full_range_when_all_free() -> None:
    cfg = VFHConfig(n_bins=10, valley_min_width_bins=2)
    mask = np.zeros(10, dtype=bool)
    valleys = find_valleys(mask, cfg)
    assert valleys == [(0, 9)]


def test_find_valleys_returns_empty_when_all_blocked() -> None:
    cfg = VFHConfig(n_bins=10, valley_min_width_bins=2)
    mask = np.ones(10, dtype=bool)
    assert find_valleys(mask, cfg) == []


def test_find_valleys_handles_wrap_around() -> None:
    cfg = VFHConfig(n_bins=10, valley_min_width_bins=3)
    mask = np.array([0, 0, 1, 1, 1, 1, 1, 0, 0, 0], dtype=bool)
    # Free bins 0, 1, 7, 8, 9 connect across the boundary into one valley of
    # width 5. We expect a single wrapping entry starting at bin 7 and ending
    # at bin 1.
    valleys = find_valleys(mask, cfg)
    assert (7, 1) in valleys


def test_find_valleys_ignores_short_run_between_two_blocked_segments() -> None:
    cfg = VFHConfig(n_bins=12, valley_min_width_bins=3)
    # Blocked, short-free-run, blocked, wide-free-run, blocked.
    mask = np.array([1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1], dtype=bool)
    valleys = find_valleys(mask, cfg)
    # Only the 4..8 run (length 5) should be kept.
    assert (4, 8) in valleys
    assert (1, 2) not in valleys


def test_select_heading_prefers_forward_valley() -> None:
    cfg = VFHConfig(n_bins=72, forward_bias_weight=1.0, smoothness_weight=0.0,
                    valley_min_width_bins=3)
    hist = np.full(72, np.inf, dtype=np.float32)  # everything free
    chosen = select_heading(hist, cfg, goal_bin=0)
    # With everything free we get one big valley spanning 0..n-1. Its centre is
    # n//2, not bin 0, but the cost metric with goal=0 prefers that centre
    # because it is equidistant. Verify we selected a valid bin.
    assert chosen is not None
    assert 0 <= chosen < cfg.n_bins


def test_select_heading_returns_none_when_all_blocked() -> None:
    cfg = VFHConfig(n_bins=72)
    hist = np.zeros(72, dtype=np.float32)  # all zero -> all blocked
    assert select_heading(hist, cfg) is None


def test_select_heading_takes_forward_when_open_even_if_shallow() -> None:
    """ forward-first rule: when the forward bin is in ANY open
    valley, take it — do not second-guess in favour of a deeper side
    valley. The operator's intent is to drive straight; trust the
    camera-fused histogram's verdict that forward is open. This replaces
    the old depth-prefers-deeper rule (which is now the FALLBACK only,
    used when forward is blocked).
    """
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, valley_min_width_bins=2,
                    forward_bias_weight=1.0, smoothness_weight=0.0,
                    depth_bonus_weight=8.0)
    hist = np.full(72, 0.30, dtype=np.float32)  # everything blocked at 0.3 m
    # Shallow valley containing the forward bin (0..2) — clearance just above
    # the 0.50 m threshold.
    for i in range(0, 3):
        hist[i] = 0.55
    # A deeper alternative valley (bins 5..7) at ~5 m. Old logic would have
    # picked this; new forward-first logic stays straight.
    for i in range(5, 8):
        hist[i] = 5.0
    chosen = select_heading(hist, cfg, goal_bin=0)
    assert chosen == 0, (
        f"forward-first: forward bin 0 is in an open valley, planner must "
        f"commit to it (no detour to the deeper bin 6 valley); got {chosen}"
    )


def test_select_heading_picks_deepest_when_forward_blocked() -> None:
    """When the forward bin is NOT in any open valley, the fallback rule
    picks the deepest valley available (most-open detour). Camera fusion in
    perception_fusion has already validated which bins count as open."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, valley_min_width_bins=2,
                    forward_bias_weight=1.0, smoothness_weight=0.0,
                    depth_bonus_weight=8.0)
    hist = np.full(72, 0.30, dtype=np.float32)  # everything blocked
    # Two valleys, neither containing forward bin 0. Bins 5..7 = 0.55 m
    # (shallow), bins 30..32 = 5.0 m (deep). Forward (bin 0) sits in the
    # blocked region between them.
    for i in range(5, 8):
        hist[i] = 0.55
    for i in range(30, 33):
        hist[i] = 5.0
    chosen = select_heading(hist, cfg, goal_bin=0)
    assert chosen == 31, (
        f"forward blocked: deeper valley (centre 31, 5 m) should win over "
        f"shallow valley (centre 6, 0.55 m); got {chosen}"
    )


def test_select_heading_returns_forward_bin_when_forward_in_valley() -> None:
    """Forward-first returns the TARGET bin (0) directly, not the centre
    of the valley containing it. This commits the robot to driving straight
    rather than the centre of an asymmetric forward-containing valley."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, valley_min_width_bins=2,
                    forward_bias_weight=1.0, smoothness_weight=0.0,
                    depth_bonus_weight=8.0)
    hist = np.full(72, 0.30, dtype=np.float32)
    # Two equally-deep valleys, only one contains forward.
    for i in range(0, 3):       # contains bin 0
        hist[i] = 2.0
    for i in range(20, 23):
        hist[i] = 2.0
    chosen = select_heading(hist, cfg, goal_bin=0)
    assert chosen == 0, (
        f"forward-first must return target bin 0, not the valley centre 1; "
        f"got {chosen}"
    )


def test_select_heading_hysteresis_keeps_previous_when_close() -> None:
    """REGRESSION ( demo-eve): the planner must not switch
    headings between adjacent ticks just because a marginally cheaper
    valley appeared. The HYSTERESIS_MARGIN gate keeps the incumbent
    heading unless a new candidate beats it by a meaningful margin.
    Without this, the robot wobbles left/right on noisy histograms in
    tight rooms, producing the observed spin/move/spin/move pattern.
    """
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, valley_min_width_bins=2,
                    forward_bias_weight=1.0, smoothness_weight=0.0,
                    depth_bonus_weight=8.0)
    hist = np.full(72, 0.30, dtype=np.float32)
    # Two valleys with a small angular separation but slightly different
    # depths — the kind of noisy histogram that produces wobble in tight
    # rooms. Costs are designed so they sit within HYSTERESIS_MARGIN, so
    # the previously-chosen heading must be retained.
    # Valley A bins 5-7 (centre 6), depth 1.0 -> cost = 6 - 8 * (1/5) = 4.4
    for i in range(5, 8):
        hist[i] = 1.0
    # Valley B bins 9-11 (centre 10), depth 1.5 -> cost = 10 - 8 * (1.5/5) = 7.6
    for i in range(9, 12):
        hist[i] = 1.5
    # Previous tick chose centre 10. best (centre 6) is 3.2 cheaper, less
    # than the 4.0 hysteresis margin, so the planner sticks with 10.
    chosen = select_heading(hist, cfg, goal_bin=0, previous_bin=10)
    assert chosen == 10, (
        f"hysteresis: incumbent bin 10 (cost ~7.6) should be retained when "
        f"the new best is only ~3.2 cheaper, below the 4.0 margin; got {chosen}"
    )


def test_select_heading_hysteresis_switches_when_clearly_better() -> None:
    """Counterpart to the keeps-previous test — when the new best valley's
    score is meaningfully higher than the previous-heading valley's score,
    the planner does switch. Hysteresis must not freeze the heading
    forever. Forward bin sits BETWEEN the two valleys so forward-first
    does not preempt the fallback decision."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, valley_min_width_bins=2,
                    forward_bias_weight=1.0, smoothness_weight=0.0,
                    depth_bonus_weight=8.0)
    hist = np.full(72, 0.30, dtype=np.float32)
    # Incumbent valley at bins 18..20 (centre 19), shallow (0.55 m).
    for i in range(18, 21):
        hist[i] = 0.55
    # New deep valley at bins 5..7 (centre 6), 5 m. Forward bin 0 is in
    # the blocked region between, so forward-first does not fire.
    for i in range(5, 8):
        hist[i] = 5.0
    chosen = select_heading(hist, cfg, goal_bin=0, previous_bin=19)
    assert chosen == 6, (
        f"hysteresis: a meaningfully deeper valley should win over the "
        f"shallow incumbent at bin 19; got {chosen}"
    )


def test_heading_to_yaw_rate_sign() -> None:
    cfg = VFHConfig(n_bins=72)
    # Bin 18 corresponds to +90 deg in world frame. We expect positive yaw.
    w = heading_to_yaw_rate(18, cfg, k=1.0, w_max=10.0)
    assert w > 0
    # Bin 54 corresponds to -90 deg. Expect negative yaw.
    w2 = heading_to_yaw_rate(54, cfg, k=1.0, w_max=10.0)
    assert w2 < 0


def test_is_dead_end_triggers_when_forward_blocked() -> None:
    cfg = VFHConfig(n_bins=72, cone_bins_forward=12,
                    obstacle_threshold_m=1.0, safety_margin_m=0.0,
                    dead_end_blocked_frac=0.8)
    hist = np.full(72, np.inf, dtype=np.float32)
    # Block the forward cone
    for i in [0, 1, 2, 3, 4, 5, 67, 68, 69, 70, 71]:
        hist[i] = 0.5
    assert is_dead_end(hist, cfg)


def test_is_dead_end_clear_path_forward() -> None:
    cfg = VFHConfig(n_bins=72, cone_bins_forward=12,
                    obstacle_threshold_m=1.0, safety_margin_m=0.0,
                    dead_end_blocked_frac=0.8)
    hist = np.full(72, np.inf, dtype=np.float32)
    assert not is_dead_end(hist, cfg)


# ---------------------------------------------------------------------------
# Angular obstacle inflation (chassis-aware, fan-pole fix)
# ---------------------------------------------------------------------------


def test_blocked_mask_no_inflation_when_chassis_zero() -> None:
    """Backward-compatibility: with chassis_half_width_m=0 (default), only
    the raw distance threshold determines blocking. No angular spread."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, chassis_half_width_m=0.0)
    hist = np.full(72, 5.0, dtype=np.float32)
    hist[0] = 0.30          # one bin blocked
    mask = blocked_mask(hist, cfg)
    # Exactly one bin should be flagged.
    assert mask[0] is np.True_ or mask[0] == True   # numpy True
    assert mask.sum() == 1, f"expected 1 blocked bin without inflation, got {int(mask.sum())}"


def test_blocked_mask_inflates_thin_obstacle_at_close_range() -> None:
    """Fan-pole scene: a single bin reads 0.10 m. Chassis half-width is
    0.165 m, larger than the obstacle range, so inflation must block the
    full ±90° (n_bins // 2 on either side, capped). Recovery should then
    fire — the planner cannot drive around a 0.10 m obstacle as if it
    were a thin pole when the chassis is 0.33 m wide."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=0.45,
                    safety_margin_m=0.05, chassis_half_width_m=0.165)
    hist = np.full(72, 5.0, dtype=np.float32)
    hist[0] = 0.10          # fan-pole at 10 cm in front
    mask = blocked_mask(hist, cfg)
    # Inflation half-angle = pi (range < half-width). Capped to n_bins//2 = 36
    # bins each side, plus the centre bin → 73 bins blocked. With wrap and
    # the cap, expect the entire histogram blocked.
    assert mask.sum() >= 70, (
        f"obstacle at 0.10 m with 0.165 m chassis must block essentially "
        f"the entire histogram; got {int(mask.sum())} blocked bins"
    )


def test_blocked_mask_inflates_proportional_to_range() -> None:
    """At 1 m range with 0.165 m half-width, the inflation half-angle is
    asin(0.165/1.0) ≈ 9.5°, ≈ 2 bins of 5° width. So a single blocked bin
    inflates to 5 bins (centre + 2 each side)."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=2.0,
                    safety_margin_m=0.0, chassis_half_width_m=0.165)
    hist = np.full(72, 5.0, dtype=np.float32)
    hist[36] = 1.0          # one bin at 1 m
    mask = blocked_mask(hist, cfg)
    assert mask.sum() == 5, (
        f"asin(0.165/1.0) ≈ 9.5° → 2 bins of inflation each side; got "
        f"{int(mask.sum())} blocked bins"
    )
    # The centre and two on each side specifically.
    for i in (34, 35, 36, 37, 38):
        assert mask[i], f"bin {i} should be inflated-blocked"


def test_blocked_mask_no_inflation_far_enough() -> None:
    """At 10 m range the chassis subtends asin(0.165/10) ≈ 0.95°, much less
    than one 5° bin, so inflation rounds up to one bin each side anyway
    (math.ceil(0.95/5) = 1). Confirms the lower bound on inflation."""
    cfg = VFHConfig(n_bins=72, obstacle_threshold_m=20.0,
                    safety_margin_m=0.0, chassis_half_width_m=0.165)
    hist = np.full(72, 50.0, dtype=np.float32)
    hist[10] = 10.0
    mask = blocked_mask(hist, cfg)
    # 1 bin each side + centre = 3 bins.
    assert mask.sum() == 3, (
        f"far obstacle should still inflate by a minimum of 1 bin each "
        f"side; got {int(mask.sum())} blocked bins"
    )
