"""Pure-function hand-pose classifier — no ROS, no MediaPipe.

Replaces the body-pose classifier with a
single-hand classifier sized for a sit-down operator at ~0.6–1.0 m from the
robot's camera. The geometric reversal of — sitting brings the hand
into MediaPipe Hands' working sweet spot — is documented in
``.

Vocabulary (7 single-hand poses; mirrors the QR verb set 1:1 for the
cross-modality ablation). revision: STOP moved to closed fist
and the directional turns moved to a tilted open palm, after the operator
reported the original index-pointing form was hard to hold steady at
sit-down distance.

 STOP closed fist (punch) — fingers curled, thumb folded
 GO OK sign — thumb tip touches index tip forming a circle,
 with middle, ring, and pinky extended
 TURN_LEFT open palm tilted toward image-left
 TURN_RIGHT open palm tilted toward image-right
 U_TURN peace sign — index + middle extended, ring + pinky curled
 SPEED_UP thumbs up — thumb extended above the MCP, fingers curled
 SPEED_DOWN thumbs down — thumb extended below the MCP, fingers curled

EMERGENCY_STOP is intentionally absent; closed-fist STOP at priority 85
already beats reactive avoidance at 70, which is what the operator's
hand-up reaction needs to cause. See in the deviation catalog.

Landmark coordinates use MediaPipe's convention: normalised [0, 1] with
y growing DOWN the image. MediaPipe Hands gives 21 landmarks per hand. We
treat the input as a single primary hand (the largest detection in the
classifier wrapper); multi-hand handling lives in the ROS detector node.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


Gesture = Literal[
 "STOP", "GO",
 "TURN_LEFT", "TURN_RIGHT", "U_TURN",
 "SPEED_UP", "SPEED_DOWN",
 # Kept in the literal so interpreter_core still resolves these labels.
 "EMERGENCY_STOP", "RESUME", "SLOW_DOWN",
]


# MediaPipe Hands landmark indices.
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12
RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20

# Indexed by (mcp, pip, tip). Used to decide "finger extended" vs "finger curled".
# Thumb has different kinematics so it gets its own helpers below.
FINGERS = {
 "index": (INDEX_MCP, INDEX_PIP, INDEX_TIP),
 "middle": (MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP),
 "ring": (RING_MCP, RING_PIP, RING_TIP),
 "pinky": (PINKY_MCP, PINKY_PIP, PINKY_TIP),
}

# Decision thresholds (normalised image units). refactor:
# finger_extended / finger_curled now use the *Euclidean distance* from MCP
# to TIP rather than the y-only delta. Distance is invariant to hand
# orientation, so an extended finger registers correctly whether the palm
# is held vertically (camera looks up at a standing operator), horizontally
# (operator holds the hand across the body at sit-down camera height), or
# inverted (the natural thumbs-down pose). The y-only check used previously
# only worked for the vertical-palm case; everything else silently failed.
#
# At MediaPipe Hands sit-down range (~0.7 m to a 720 p frame), an extended
# finger spans ~0.18-0.25 of frame; a curled finger sits ~0.04-0.07 from
# MCP. The two thresholds leave a deliberate gap so half-extended fingers
# fall in neither bucket.
_FINGER_EXTENDED_DIST: float = 0.10
_FINGER_CURLED_DIST: float = 0.07
_FINGER_EXTENSION_MARGIN: float = 0.02 # kept for thumb helpers
# Min visibility (per-landmark confidence). MediaPipe Hands returns landmark
# scores via the post-processing wrapper; we accept slightly lower than the
# pose model used because hand tracking is naturally noisier.
MIN_VISIBILITY: float = 0.5


@dataclass
class HandFrame:
 """One MediaPipe Hands observation (single hand). The ROS wrapper picks
 the largest detection and packs it into this shape before classifying."""
 t: float
 landmarks: list[tuple[float, float, float]] # 21 × (x, y, z) normalised
 visibility: list[float] # 21 × confidence in [0, 1]
 handedness: Literal["left", "right", "unknown"] = "unknown"


@dataclass
class DetectorConfig:
 """Stability gate parameters. Carried as part of the frame plumbing so
 the ROS wrapper can override per-deployment."""
 hold_seconds: float = 0.3
 cooldown_seconds: float = 1.0


# ---------------------------------------------------------------------------
# Geometric helpers
# ---------------------------------------------------------------------------


def _x(lm: list[tuple[float, float, float]], i: int) -> float:
 return lm[i][0]


def _y(lm: list[tuple[float, float, float]], i: int) -> float:
 return lm[i][1]


def _all_visible(vis: list[float], *indices: int) -> bool:
 return all(vis[i] >= MIN_VISIBILITY for i in indices if i < len(vis))


def _mcp_tip_distance(lm, finger: str) -> float:
 """Euclidean distance from a finger's MCP to its TIP (normalised image
 units). Used as the orientation-invariant proxy for finger extension."""
 if finger not in FINGERS:
 raise ValueError(f"unknown finger: {finger!r}")
 mcp_idx, _pip_idx, tip_idx = FINGERS[finger]
 dx = _x(lm, tip_idx) - _x(lm, mcp_idx)
 dy = _y(lm, tip_idx) - _y(lm, mcp_idx)
 return (dx * dx + dy * dy) ** 0.5


def finger_extended(lm, finger: str, *, threshold: float = _FINGER_EXTENDED_DIST) -> bool:
 """True when the finger is straightened away from its MCP by more than
 ``threshold`` (normalised image units). Orientation-invariant: works
 for vertical, horizontal, and inverted hand poses."""
 return _mcp_tip_distance(lm, finger) > threshold


def finger_curled(lm, finger: str, *, threshold: float = _FINGER_CURLED_DIST) -> bool:
 """True when the finger's tip has folded back to within ``threshold``
 of its MCP — the curled-into-the-palm position. Same orientation
 invariance as ``finger_extended``."""
 return _mcp_tip_distance(lm, finger) < threshold


def thumb_extended_up(lm, *, margin: float = _FINGER_EXTENSION_MARGIN) -> bool:
 """Thumb tip above MCP by ``margin``. Used for the thumbs-up pose."""
 return _y(lm, THUMB_TIP) < _y(lm, THUMB_MCP) - margin


def thumb_extended_down(lm, *, margin: float = _FINGER_EXTENSION_MARGIN) -> bool:
 """Thumb tip below MCP by ``margin``. Used for the thumbs-down pose."""
 return _y(lm, THUMB_TIP) > _y(lm, THUMB_MCP) + margin


def thumb_tucked(lm, *, margin: float = _FINGER_EXTENSION_MARGIN) -> bool:
 """Thumb tip lies near the index MCP — i.e. the thumb has rotated across
 the palm (the closed-fist position). Confirmed by a small Euclidean
 radius around INDEX_MCP rather than only an x-axis check, so a thumbs-
 down hand (tip far below the palm) does not register as tucked."""
 dx = _x(lm, THUMB_TIP) - _x(lm, INDEX_MCP)
 dy = _y(lm, THUMB_TIP) - _y(lm, INDEX_MCP)
 return (dx * dx + dy * dy) ** 0.5 < 0.08


def all_fingers_extended(lm) -> bool:
 return all(finger_extended(lm, name) for name in FINGERS)


def all_fingers_curled(lm) -> bool:
 return all(finger_curled(lm, name) for name in FINGERS)


def fingers_mostly_curled(lm, *, min_count: int = 3) -> bool:
 """Permissive curl check used by SPEED_UP / SPEED_DOWN. Real thumbs-up
 and thumbs-down often leave one finger (typically the pinky) only
 half-curled, which broke the all-four-curled requirement under live
 testing. Accepting ≥3 of 4 curled keeps the gesture detectable without
 breaking the closed-fist disambiguation, because closed_fist still
 requires the strict all-four check plus the tucked thumb."""
 curled = sum(1 for name in FINGERS if finger_curled(lm, name))
 return curled >= min_count


def index_pointing(lm) -> bool:
 """Index extended, middle/ring/pinky curled. Direction (left/right) is
 handled by the higher-level ``index_pointing_left/right`` helpers."""
 return (
 finger_extended(lm, "index")
 and finger_curled(lm, "middle")
 and finger_curled(lm, "ring")
 and finger_curled(lm, "pinky")
 )


def index_pointing_left(lm) -> bool:
 """Index pointing image-left: tip x significantly less than wrist x."""
 return index_pointing(lm) and (_x(lm, INDEX_TIP) < _x(lm, WRIST) - 0.05)


def index_pointing_right(lm) -> bool:
 """Index pointing image-right: tip x significantly greater than wrist x."""
 return index_pointing(lm) and (_x(lm, INDEX_TIP) > _x(lm, WRIST) + 0.05)


def peace_sign(lm) -> bool:
 """Index AND middle extended, ring AND pinky curled."""
 return (
 finger_extended(lm, "index")
 and finger_extended(lm, "middle")
 and finger_curled(lm, "ring")
 and finger_curled(lm, "pinky")
 )


# OK sign — thumb tip and index tip touching (forming a circle), with the
# remaining three fingers extended. The "touching" check uses Euclidean
# distance in normalised coordinates; ~0.05 is loose enough to tolerate
# MediaPipe Hands' tracking noise but tight enough to distinguish the OK
# sign from a relaxed-thumb pose where the thumb is merely near the index.
_OK_TOUCH_RADIUS: float = 0.06


def ok_sign(lm) -> bool:
 """OK gesture: thumb tip near index tip; middle, ring, and pinky
 fingers extended away from the palm."""
 dx = _x(lm, THUMB_TIP) - _x(lm, INDEX_TIP)
 dy = _y(lm, THUMB_TIP) - _y(lm, INDEX_TIP)
 thumb_index_close = (dx * dx + dy * dy) ** 0.5 < _OK_TOUCH_RADIUS
 others_extended = (
 finger_extended(lm, "middle")
 and finger_extended(lm, "ring")
 and finger_extended(lm, "pinky")
 )
 return thumb_index_close and others_extended


def closed_fist(lm) -> bool:
 """Punch — all four fingers curled into the palm AND the thumb folded
 across or onto the palm. The thumb-tucked guard rejects loose half-
 closed hands and a thumbs-up that happens to be poorly held."""
 return all_fingers_curled(lm) and thumb_tucked(lm)


# Direction threshold for the open-palm pointing turn cue. The
# rev abandoned the vertical-palm-tilted-sideways rule because the operator
# at sit-down camera height naturally holds the hand HORIZONTALLY across
# the body, fingers projecting to one side — see the IMG_4153/IMG_4154
# The rule below requires the
# wrist→middle-MCP axis to be predominantly horizontal (|dx| > |dy|).# the middle-MCP at least _PALM_POINT_MIN of frame width offset from the
# wrist in the cued direction.
_PALM_POINT_MIN: float = 0.06


def palm_pointing_left(lm) -> bool:
 """Open palm with all fingers extended, fingers projecting toward
 image-LEFT. The hand may be perfectly horizontal or tilted up to 45°
 from horizontal — the rule allows any direction where the |dx| of the
 wrist→middle-MCP vector dominates the |dy|."""
 if not all_fingers_extended(lm):
 return False
 dx = _x(lm, MIDDLE_MCP) - _x(lm, WRIST)
 dy = _y(lm, MIDDLE_MCP) - _y(lm, WRIST)
 if dx > -_PALM_POINT_MIN: # not enough offset to image-left
 return False
 return abs(dx) > abs(dy) # predominantly horizontal


def palm_pointing_right(lm) -> bool:
 """Open palm with all fingers extended, fingers projecting toward
 image-RIGHT (mirror of the left rule)."""
 if not all_fingers_extended(lm):
 return False
 dx = _x(lm, MIDDLE_MCP) - _x(lm, WRIST)
 dy = _y(lm, MIDDLE_MCP) - _y(lm, WRIST)
 if dx < _PALM_POINT_MIN:
 return False
 return abs(dx) > abs(dy)


def gun_pointing_left(lm) -> bool:
 """"Gun" pose pointing image-LEFT — all four fingers extended toward
 image-left, thumb extended UP (away from the palm). The thumb-up cue
 acts as an orientation anchor that is much harder to mis-fire than the
 bare palm-pointing rule, because no other vocabulary pose holds the
 thumb up while the other four fingers are extended (SPEED_UP also has
 thumb up but its fingers are curled). Operator-recommended at the
 demo-eve session —."""
 if not all_fingers_extended(lm):
 return False
 if not thumb_extended_up(lm):
 return False
 if thumb_tucked(lm):
 return False
 dx = _x(lm, MIDDLE_MCP) - _x(lm, WRIST)
 return dx < -_PALM_POINT_MIN


def gun_pointing_right(lm) -> bool:
 """Mirror of ``gun_pointing_left`` — fingers project image-RIGHT,
 thumb still extended up."""
 if not all_fingers_extended(lm):
 return False
 if not thumb_extended_up(lm):
 return False
 if thumb_tucked(lm):
 return False
 dx = _x(lm, MIDDLE_MCP) - _x(lm, WRIST)
 return dx > _PALM_POINT_MIN


# ---------------------------------------------------------------------------
# Classification entry point
# ---------------------------------------------------------------------------


def classify(
 hand: HandFrame | None,
 history: dict | None = None, # unused — signature stability
 cfg: DetectorConfig | None = None, # unused
) -> Gesture | None:
 """Map one hand frame to a vocabulary label, or None if no rule matches.

 Order matters: more specific patterns are tested before less specific
 ones. Peace sign comes before STOP because peace has two extended
 fingers and would otherwise be partially absorbed by an open-palm rule.
 """
 del history, cfg
 if hand is None or len(hand.landmarks) < 21:
 return None
 if len(hand.visibility) < 21:
 return None
 lm, vis = hand.landmarks, hand.visibility
 if not _all_visible(vis, WRIST, INDEX_MCP, INDEX_TIP, MIDDLE_TIP, THUMB_TIP):
 return None

 # 1. Peace sign — most distinctive (exactly two adjacent fingers up).
 if peace_sign(lm):
 return "U_TURN"

 # 2. Thumbs up / down — thumb extended vertically AND held outside the
 # palm so it cannot be confused with a fist. The "not tucked" guard
 # distinguishes a clear thumbs-up from a relaxed fist whose thumb is
 # folded across the palm. Curl check is "mostly curled" (≥3 of 4) so
 # a half-curled pinky does not break detection — see
 # fingers_mostly_curled rationale.
 fingers_curled_strict = all_fingers_curled(lm)
 fingers_curled_loose = fingers_mostly_curled(lm)
 thumb_clear = not thumb_tucked(lm)
 if fingers_curled_loose and thumb_clear and thumb_extended_up(lm):
 return "SPEED_UP"
 if fingers_curled_loose and thumb_clear and thumb_extended_down(lm):
 return "SPEED_DOWN"

 # 3. OK sign — thumb tip touches index tip; middle/ring/pinky extended.
 # This replaces the closed-fist GO from earlier revisions; the OK sign
 # is unambiguous and will not be triggered by a relaxed idle hand. See
 # for the rationale.
 if ok_sign(lm):
 return "GO"

 # 4. Direction (TURN_LEFT / TURN_RIGHT). Two acceptable forms, checked
 # in order of robustness:
 #
 # (a) "Gun" pose — fingers extended in direction + thumb extended UP.
 # The thumb-up acts as an orientation anchor; this pose almost
 # never mis-fires because no other vocabulary pose has thumb-up
 # AND fingers-extended at the same time.
 # (b) Bare horizontal palm — fingers extended pointing to one side
 # with the wrist→middle-MCP axis predominantly horizontal. Kept
 # as a fallback for the operator who forgets the thumb-up anchor.
 #
 # Both forms map to the same verb, so the OR is safe; (a) just gets
 # higher precedence so we don't accidentally fall through to (b) in
 # ambiguous frames.
 if gun_pointing_left(lm):
 return "TURN_LEFT"
 if gun_pointing_right(lm):
 return "TURN_RIGHT"
 if palm_pointing_left(lm):
 return "TURN_LEFT"
 if palm_pointing_right(lm):
 return "TURN_RIGHT"

 # 5. Closed fist — STOP. Replaces the older open-palm STOP.
 # Last in the chain because it is the lowest-information pose and would
 # otherwise absorb half-closed transitional hands; the strict all-four-
 # curled check (via closed_fist) keeps the relaxed thumbs-up/down rule
 # above from leaking into STOP, and the thumb-tucked guard rejects
 # accidental thumbs-up classifications.
 if fingers_curled_strict and thumb_tucked(lm):
 return "STOP"

 return None
