"""Contract tests for the hand-pose gesture classifier (post
rebuild from MediaPipe Pose to MediaPipe Hands).

Synthesises 21-landmark hand layouts so MediaPipe is not required at test
time. Image y grows down; coordinates are normalised [0, 1].

Mental model: the operator sits ~0.7 m in front of the camera with the
right hand raised. The wrist is roughly mid-frame; fingers above the wrist
extend toward smaller y. Wrist x is roughly 0.5 by default and individual
gestures shift it as needed.
"""
from __future__ import annotations

from par_gesture.classifier_core import (
    FINGERS,
    INDEX_MCP,
    INDEX_PIP,
    INDEX_TIP,
    MIDDLE_MCP,
    PINKY_MCP,
    RING_MCP,
    THUMB_CMC,
    THUMB_IP,
    THUMB_MCP,
    THUMB_TIP,
    WRIST,
    HandFrame,
    classify,
    closed_fist,
    finger_curled,
    finger_extended,
    index_pointing,
    gun_pointing_left,
    gun_pointing_right,
    palm_pointing_left,
    palm_pointing_right,
    peace_sign,
    thumb_extended_down,
    thumb_extended_up,
    thumb_tucked,
)


def _empty_hand() -> list[tuple[float, float, float]]:
    return [(0.5, 0.5, 0.0) for _ in range(21)]


def _full_vis() -> list[float]:
    return [0.99 for _ in range(21)]


def _set(lm, idx, x, y, z=0.0):
    lm[idx] = (x, y, z)


def _palm_anchor(lm) -> None:
    """Lay down a wrist + the four MCPs in a reasonable palm geometry."""
    _set(lm, WRIST, 0.5, 0.60)
    _set(lm, INDEX_MCP, 0.55, 0.50)
    _set(lm, MIDDLE_MCP, 0.50, 0.50)
    _set(lm, RING_MCP, 0.45, 0.50)
    _set(lm, PINKY_MCP, 0.40, 0.50)
    _set(lm, THUMB_CMC, 0.58, 0.55)
    _set(lm, THUMB_MCP, 0.62, 0.50)
    _set(lm, THUMB_IP, 0.65, 0.45)


def _extend_finger(lm, finger: str, *, tip_y: float | None = None) -> None:
    mcp_idx, pip_idx, tip_idx = FINGERS[finger]
    mcp_x, mcp_y, _ = lm[mcp_idx]
    if tip_y is None:
        tip_y = mcp_y - 0.20
    pip_y = mcp_y - 0.10
    dip_idx = pip_idx + 1
    _set(lm, pip_idx, mcp_x, pip_y)
    _set(lm, dip_idx, mcp_x, (pip_y + tip_y) / 2)
    _set(lm, tip_idx, mcp_x, tip_y)


def _curl_finger(lm, finger: str) -> None:
    mcp_idx, pip_idx, tip_idx = FINGERS[finger]
    mcp_x, mcp_y, _ = lm[mcp_idx]
    pip_y = mcp_y - 0.08
    tip_y = mcp_y + 0.02
    dip_idx = pip_idx + 1
    _set(lm, pip_idx, mcp_x, pip_y)
    _set(lm, dip_idx, mcp_x, mcp_y - 0.02)
    _set(lm, tip_idx, mcp_x, tip_y)


def _open_palm() -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    for finger in FINGERS:
        _extend_finger(lm, finger)
    _set(lm, THUMB_TIP, 0.72, 0.40)   # extended sideways
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _closed_fist() -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    for finger in FINGERS:
        _curl_finger(lm, finger)
    _set(lm, THUMB_TIP, 0.55, 0.55)   # tucked across the palm
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _ok_sign() -> HandFrame:
    """OK gesture — thumb tip touches index tip, middle/ring/pinky extended."""
    lm = _empty_hand()
    _palm_anchor(lm)
    _curl_finger(lm, "index")
    # Place INDEX_TIP and THUMB_TIP touching at one point (forming the loop).
    _set(lm, INDEX_TIP, 0.50, 0.42)
    _set(lm, THUMB_TIP, 0.50, 0.42)
    _extend_finger(lm, "middle")
    _extend_finger(lm, "ring")
    _extend_finger(lm, "pinky")
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _peace_sign() -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    _extend_finger(lm, "index")
    _extend_finger(lm, "middle")
    _curl_finger(lm, "ring")
    _curl_finger(lm, "pinky")
    _set(lm, THUMB_TIP, 0.55, 0.55)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _index_point(direction: str) -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    _extend_finger(lm, "index")
    if direction == "left":
        _set(lm, INDEX_TIP, 0.30, 0.30)
    else:
        _set(lm, INDEX_TIP, 0.70, 0.30)
    _curl_finger(lm, "middle")
    _curl_finger(lm, "ring")
    _curl_finger(lm, "pinky")
    _set(lm, THUMB_TIP, 0.55, 0.55)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _palm_horizontal(direction: str) -> HandFrame:
    """Open palm held horizontally across the body, fingers projecting to
    one image side. Mirrors the operator's natural sit-down pose
 (IMG_4154). The wrist sits at
    one image side and the MCPs run across the frame at roughly the same y."""
    lm = _empty_hand()
    if direction == "left":
        # Wrist at right, fingers point image-LEFT.
        _set(lm, WRIST, 0.75, 0.50)
        _set(lm, INDEX_MCP, 0.55, 0.45)
        _set(lm, MIDDLE_MCP, 0.55, 0.50)
        _set(lm, RING_MCP, 0.55, 0.55)
        _set(lm, PINKY_MCP, 0.55, 0.60)
        # Each finger's TIP placed further toward image-left.
        _set(lm, INDEX_TIP, 0.30, 0.45)
        _set(lm, MIDDLE_TIP := 12, 0.28, 0.50)
        _set(lm, RING_TIP := 16, 0.30, 0.55)
        _set(lm, PINKY_TIP := 20, 0.32, 0.60)
        # PIPs roughly midway so finger_extended (distance ≥ 0.10) passes.
        _set(lm, INDEX_PIP, 0.42, 0.45)
        _set(lm, 10, 0.40, 0.50)   # MIDDLE_PIP
        _set(lm, 14, 0.42, 0.55)   # RING_PIP
        _set(lm, 18, 0.44, 0.60)   # PINKY_PIP
        _set(lm, THUMB_CMC, 0.78, 0.55)
        _set(lm, THUMB_MCP, 0.74, 0.60)
        _set(lm, THUMB_IP, 0.70, 0.62)
        _set(lm, THUMB_TIP, 0.66, 0.65)
    else:
        # Wrist at left, fingers point image-RIGHT.
        _set(lm, WRIST, 0.25, 0.50)
        _set(lm, INDEX_MCP, 0.45, 0.45)
        _set(lm, MIDDLE_MCP, 0.45, 0.50)
        _set(lm, RING_MCP, 0.45, 0.55)
        _set(lm, PINKY_MCP, 0.45, 0.60)
        _set(lm, INDEX_TIP, 0.70, 0.45)
        _set(lm, 12, 0.72, 0.50)
        _set(lm, 16, 0.70, 0.55)
        _set(lm, 20, 0.68, 0.60)
        _set(lm, INDEX_PIP, 0.58, 0.45)
        _set(lm, 10, 0.60, 0.50)
        _set(lm, 14, 0.58, 0.55)
        _set(lm, 18, 0.56, 0.60)
        _set(lm, THUMB_CMC, 0.22, 0.55)
        _set(lm, THUMB_MCP, 0.26, 0.60)
        _set(lm, THUMB_IP, 0.30, 0.62)
        _set(lm, THUMB_TIP, 0.34, 0.65)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _thumbs_up() -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    for finger in FINGERS:
        _curl_finger(lm, finger)
    # Thumb extended UP, kept off the index column to avoid the tucked check.
    _set(lm, THUMB_MCP, 0.62, 0.50)
    _set(lm, THUMB_IP, 0.62, 0.40)
    _set(lm, THUMB_TIP, 0.62, 0.30)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def _thumbs_down() -> HandFrame:
    lm = _empty_hand()
    _palm_anchor(lm)
    for finger in FINGERS:
        _curl_finger(lm, finger)
    _set(lm, THUMB_MCP, 0.62, 0.50)
    _set(lm, THUMB_IP, 0.62, 0.60)
    _set(lm, THUMB_TIP, 0.62, 0.70)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


# ---------------------------------------------------------------------------
# Helper unit tests
# ---------------------------------------------------------------------------


def test_finger_extended_when_tip_above_pip() -> None:
    hand = _open_palm()
    for finger in FINGERS:
        assert finger_extended(hand.landmarks, finger), finger


def test_finger_curled_when_tip_below_pip() -> None:
    hand = _closed_fist()
    for finger in FINGERS:
        assert finger_curled(hand.landmarks, finger), finger


def test_thumb_extended_up_for_thumbs_up() -> None:
    assert thumb_extended_up(_thumbs_up().landmarks) is True
    assert thumb_extended_down(_thumbs_up().landmarks) is False


def test_thumb_extended_down_for_thumbs_down() -> None:
    assert thumb_extended_down(_thumbs_down().landmarks) is True
    assert thumb_extended_up(_thumbs_down().landmarks) is False


def test_thumb_tucked_for_closed_fist() -> None:
    assert thumb_tucked(_closed_fist().landmarks) is True


def test_thumb_not_tucked_for_open_palm() -> None:
    assert thumb_tucked(_open_palm().landmarks) is False


def test_peace_sign_helper() -> None:
    assert peace_sign(_peace_sign().landmarks) is True


def test_index_pointing_only_index_extended() -> None:
    assert index_pointing(_index_point("left").landmarks) is True
    assert index_pointing(_open_palm().landmarks) is False


# ---------------------------------------------------------------------------
# classify() — one test per verb plus the negative cases
# ---------------------------------------------------------------------------


def test_classify_closed_fist_is_stop() -> None:
    """ vocabulary: STOP is now the closed fist (punch). Replaces
    the older open-palm STOP after operator feedback that the open hand
    was awkward to hold steady at sit-down distance."""
    assert classify(_closed_fist()) == "STOP"


def test_classify_open_palm_vertical_is_not_stop() -> None:
    """A vertical open palm no longer maps to STOP. It is reserved as the
    base pose for the tilted-palm directional turns; classifier abstains
    when neither tilt threshold is reached."""
    assert classify(_open_palm()) != "STOP"


def test_classify_ok_sign_is_go() -> None:
    """The OK gesture (thumb + index forming a circle, others extended)
    is GO. Replaces the earlier closed-fist GO — the OK sign is unambiguous
 and will not be triggered by a relaxed idle hand. See
 + classifier_core.ok_sign."""
    assert classify(_ok_sign()) == "GO"


def test_classify_closed_fist_is_no_longer_go() -> None:
    """A closed fist no longer maps to GO. Post it is STOP."""
    assert classify(_closed_fist()) != "GO"


def test_classify_peace_sign_is_u_turn() -> None:
    assert classify(_peace_sign()) == "U_TURN"


def test_classify_thumbs_up_is_speed_up() -> None:
    assert classify(_thumbs_up()) == "SPEED_UP"


def test_classify_thumbs_down_is_speed_down() -> None:
    assert classify(_thumbs_down()) == "SPEED_DOWN"


def test_classify_palm_pointing_left_is_turn_left() -> None:
    """Open palm held horizontally with fingers pointing image-LEFT maps
    to TURN_LEFT. Mirrors the operator's natural sit-down pose."""
    assert classify(_palm_horizontal("left")) == "TURN_LEFT"


def test_classify_palm_pointing_right_is_turn_right() -> None:
    """Mirror of the left case — fingers pointing image-RIGHT maps to
    TURN_RIGHT."""
    assert classify(_palm_horizontal("right")) == "TURN_RIGHT"


def test_palm_pointing_helper_directions() -> None:
    """Direction helpers must be mutually exclusive for a clean horizontal
    palm — one fires, the other doesn't."""
    assert palm_pointing_left(_palm_horizontal("left").landmarks) is True
    assert palm_pointing_right(_palm_horizontal("left").landmarks) is False
    assert palm_pointing_right(_palm_horizontal("right").landmarks) is True
    assert palm_pointing_left(_palm_horizontal("right").landmarks) is False


def _gun_pose(direction: str) -> HandFrame:
    """Operator-recommended turn pose : all four fingers extended
    pointing to one image side, thumb extended UP toward image-top. The
    thumb-up anchor disambiguates this pose from any other in the vocab."""
    lm = _empty_hand()
    if direction == "left":
        # Wrist at right, fingers point image-LEFT, thumb sticks up at the
        # wrist end of the hand.
        _set(lm, WRIST, 0.75, 0.50)
        _set(lm, INDEX_MCP, 0.55, 0.45)
        _set(lm, MIDDLE_MCP, 0.55, 0.50)
        _set(lm, RING_MCP, 0.55, 0.55)
        _set(lm, PINKY_MCP, 0.55, 0.60)
        _set(lm, INDEX_TIP, 0.30, 0.45)
        _set(lm, 12, 0.28, 0.50)
        _set(lm, 16, 0.30, 0.55)
        _set(lm, 20, 0.32, 0.60)
        _set(lm, INDEX_PIP, 0.42, 0.45)
        _set(lm, 10, 0.40, 0.50)
        _set(lm, 14, 0.42, 0.55)
        _set(lm, 18, 0.44, 0.60)
        # Thumb at the wrist end, sticking UP.
        _set(lm, THUMB_CMC, 0.74, 0.48)
        _set(lm, THUMB_MCP, 0.72, 0.40)
        _set(lm, THUMB_IP, 0.72, 0.30)
        _set(lm, THUMB_TIP, 0.72, 0.20)
    else:
        _set(lm, WRIST, 0.25, 0.50)
        _set(lm, INDEX_MCP, 0.45, 0.45)
        _set(lm, MIDDLE_MCP, 0.45, 0.50)
        _set(lm, RING_MCP, 0.45, 0.55)
        _set(lm, PINKY_MCP, 0.45, 0.60)
        _set(lm, INDEX_TIP, 0.70, 0.45)
        _set(lm, 12, 0.72, 0.50)
        _set(lm, 16, 0.70, 0.55)
        _set(lm, 20, 0.68, 0.60)
        _set(lm, INDEX_PIP, 0.58, 0.45)
        _set(lm, 10, 0.60, 0.50)
        _set(lm, 14, 0.58, 0.55)
        _set(lm, 18, 0.56, 0.60)
        _set(lm, THUMB_CMC, 0.26, 0.48)
        _set(lm, THUMB_MCP, 0.28, 0.40)
        _set(lm, THUMB_IP, 0.28, 0.30)
        _set(lm, THUMB_TIP, 0.28, 0.20)
    return HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())


def test_gun_pointing_left_is_turn_left() -> None:
    """Thumb-up + fingers pointing image-left maps to TURN_LEFT via the
    high-precedence gun-pose branch."""
    assert classify(_gun_pose("left")) == "TURN_LEFT"


def test_gun_pointing_right_is_turn_right() -> None:
    """Mirror of the gun-left case."""
    assert classify(_gun_pose("right")) == "TURN_RIGHT"


def test_gun_pointing_helpers_mutually_exclusive() -> None:
    assert gun_pointing_left(_gun_pose("left").landmarks) is True
    assert gun_pointing_right(_gun_pose("left").landmarks) is False
    assert gun_pointing_right(_gun_pose("right").landmarks) is True
    assert gun_pointing_left(_gun_pose("right").landmarks) is False


def test_gun_pose_does_not_collide_with_speed_up() -> None:
    """SPEED_UP requires the four fingers CURLED, the gun pose has them
    EXTENDED — the order of checks must not let SPEED_UP swallow a gun
    pose by accident."""
    assert classify(_gun_pose("left")) != "SPEED_UP"
    assert classify(_gun_pose("right")) != "SPEED_UP"


def test_closed_fist_helper() -> None:
    assert closed_fist(_closed_fist().landmarks) is True
    assert closed_fist(_open_palm().landmarks) is False


def test_speed_up_tolerates_one_uncurled_finger() -> None:
    """Real thumbs-up often leaves the pinky half-extended. The
    rule loosens the curl check to ≥3 of 4, so one stray finger does not
    break detection."""
    hand = _thumbs_up()
    lm = list(hand.landmarks)
    _extend_finger(lm, "pinky")  # pinky pops out
    hand = HandFrame(t=0.0, landmarks=lm, visibility=hand.visibility)
    assert classify(hand) == "SPEED_UP"


def test_speed_down_tolerates_one_uncurled_finger() -> None:
    """Mirror of the SPEED_UP relaxation — one half-extended pinky on a
    thumbs-down hand still classifies as SPEED_DOWN."""
    hand = _thumbs_down()
    lm = list(hand.landmarks)
    _extend_finger(lm, "pinky")
    hand = HandFrame(t=0.0, landmarks=lm, visibility=hand.visibility)
    assert classify(hand) == "SPEED_DOWN"


def test_classify_no_hand_returns_none() -> None:
    assert classify(None) is None


def test_classify_short_landmark_list_returns_none() -> None:
    short = HandFrame(t=0.0, landmarks=[(0.5, 0.5, 0.0)] * 10,
                      visibility=[1.0] * 10)
    assert classify(short) is None


def test_classify_no_match_returns_none() -> None:
    """A hand frame that satisfies no rule should fall through to None."""
    lm = _empty_hand()
    _palm_anchor(lm)
    _extend_finger(lm, "index")
    # Override index tip to align with the wrist x — neither left nor right
    # pointer fires.
    _set(lm, INDEX_TIP, 0.50, 0.30)
    _curl_finger(lm, "middle")
    _curl_finger(lm, "ring")
    _curl_finger(lm, "pinky")
    _set(lm, THUMB_MCP, 0.62, 0.50)
    _set(lm, THUMB_TIP, 0.66, 0.48)   # neutral, neither up nor down nor tucked
    hand = HandFrame(t=0.0, landmarks=lm, visibility=_full_vis())
    assert classify(hand) is None


def test_classify_low_visibility_returns_none() -> None:
    """When key landmarks fall below MIN_VISIBILITY the classifier abstains."""
    hand = _open_palm()
    bad_vis = list(hand.visibility)
    bad_vis[INDEX_TIP] = 0.1
    hand = HandFrame(t=0.0, landmarks=hand.landmarks, visibility=bad_vis)
    assert classify(hand) is None


def test_classify_priority_peace_over_open_palm() -> None:
    """A peace sign must classify as U_TURN, not STOP, even though both
    have at least two extended fingers."""
    assert classify(_peace_sign()) == "U_TURN"


def test_classify_thumbs_up_does_not_collide_with_ok() -> None:
    """Thumbs up has all four non-thumb fingers curled and the thumb
    extended UP — clearly distinct from the OK sign (thumb touching
    index, middle/ring/pinky extended). Confirm the order-of-checks
    resolves it as SPEED_UP not GO."""
    assert classify(_thumbs_up()) == "SPEED_UP"


def test_open_palm_with_thumb_tucked_is_not_stop() -> None:
    """A vertical open palm with a tucked thumb is still not STOP under the
 vocabulary — STOP requires a closed fist. The test is kept
    to lock in the negative for any transitional half-closed hand."""
    hand = _open_palm()
    lm = list(hand.landmarks)
    _set(lm, THUMB_TIP, 0.55, 0.55)   # tuck thumb across palm
    hand = HandFrame(t=0.0, landmarks=lm, visibility=hand.visibility)
    assert classify(hand) != "STOP"
