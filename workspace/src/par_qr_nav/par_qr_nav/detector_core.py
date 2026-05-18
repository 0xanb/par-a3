"""Pure-function QR detection core — no ROS, no cv_bridge.

Kept separate from the node wrapper so unit tests can exercise the decoder
without spinning up rclpy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np


# The 7 command verbs the spec requires. Any other payload is reported as
# UNKNOWN so the command interpreter can trigger the RECOVERING state.
KNOWN_VERBS: frozenset[str] = frozenset(
 {"TURN_LEFT", "TURN_RIGHT", "STOP", "GO", "SPEED_UP", "SPEED_DOWN", "U_TURN"}
)


def normalize_payload(raw: str | None) -> str:
 """Normalise a raw QR payload before vocabulary lookup.

 Three transformations applied in order:

 1. Strip surrounding whitespace + uppercase. Most online QR generators
 append a trailing newline; many cards printed by non-spec tooling
 come out as ``"go"`` or ``"Go\\n"``. Without this step both fail the
 exact-match lookup and trigger a RECOVERING event.
 2. URL trailing-segment fallback. A QR payload of
 ``"https://example.com/STOP"`` is taken as the user's intent to send
 the verb after the last ``/``. This catches the common case where
 an operator generates QR codes through a URL-wrapping app.
 3. Returns "" when the input is None or empty post-strip; callers
 short-circuit on the empty string.
 """
 if raw is None:
 return ""
 s = raw.strip.upper
 if not s:
 return ""
 if s in KNOWN_VERBS:
 return s
 # URL fallback: take the last path segment if a slash is present.
 if "/" in s:
 tail = s.rsplit("/", 1)[-1].strip
 if tail in KNOWN_VERBS:
 return tail
 return s


@dataclass
class QRResult:
 """One decoded QR code in one frame."""
 payload: str
 # Bounding polygon: list of (x, y) in image pixels. OpenCV returns a 4x2
 # array per code (corners, clockwise from top-left).
 polygon: list[tuple[int, int]]
 # Image-space centroid of the polygon (convenience for logging / ROI gating).
 centroid: tuple[int, int]
 # True when the payload matches a known verb.
 known: bool


def _polygon_area(poly_pts) -> float:
 """Shoelace area of a (typically 4-point) polygon. Used to rank
 competing QR candidates by image-space size: a closer card produces
 a larger polygon, which is the operator's intuition for "the card
 I'm pointing at." See multi-detection edge case in the demo audit."""
 pts = np.asarray(poly_pts, dtype=np.float64).reshape(-1, 2)
 if len(pts) < 3:
 return 0.0
 x = pts[:, 0]
 y = pts[:, 1]
 return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _mk_result(payload: str, poly_pts) -> QRResult:
 poly_int = [(int(round(x)), int(round(y))) for (x, y) in poly_pts]
 cx = int(round(sum(p[0] for p in poly_int) / len(poly_int)))
 cy = int(round(sum(p[1] for p in poly_int) / len(poly_int)))
 return QRResult(
 payload=payload,
 polygon=poly_int,
 centroid=(cx, cy),
 known=payload in KNOWN_VERBS,
 )


def detect(image_bgr: np.ndarray, detector: cv2.QRCodeDetector | None = None
 ) -> list[QRResult]:
 """Detect and decode every QR code in one BGR frame.

 OpenCV's ``detectAndDecodeMulti`` is the only path that reports multiple
 codes in one frame (required by the spec's simultaneous-code edge case),
 but is measurably less sensitive than the single-code ``detectAndDecode``
 on the OAK-D Pro's 768x432 preview stream. We run the single-code path
 first for robustness and fall back to the multi path for two-code scenes.

 Payload normalisation (whitespace, case, URL-wrapping) is applied via
 ``normalize_payload`` before the result is constructed so downstream
 KNOWN_VERBS lookups behave consistently across QR-generator quirks.

 When two or more codes are decoded in the same frame, results are
 sorted by polygon area in descending order: the largest (i.e. closest-
 to-camera) card wins. The downstream temporal voter still operates on
 the order returned here, so this also affects which card "wins" the
 vote in a multi-card frame.
 """
 if image_bgr is None or image_bgr.size == 0:
 return []
 det = detector or cv2.QRCodeDetector

 raw_results: list[tuple[str, list, float]] = [] # (payload, poly, area)

 # Primary: single-code decoder. Fast, highly reliable on typical demo QRs.
 # OpenCV 4.11 raises cv2.error when a candidate QR contour has zero area
 # (a real-world artefact when the camera sees a near-edge QR-like pattern).
 # The defensive wrapper treats those as "no decode this frame" rather than
 # crashing the node —md.
 try:
 data, points, _ = det.detectAndDecode(image_bgr)
 except cv2.error:
 data, points = "", None
 if data and points is not None and len(points) > 0:
 pts = np.asarray(points).reshape(-1, 2)
 if len(pts) >= 3:
 poly_list = pts.tolist
 raw_results.append((data, poly_list, _polygon_area(poly_list)))

 # Fallback: multi-code detector catches two-code scenes and sometimes
 # recovers codes the single decoder missed. Same defensive wrapper.
 try:
 ok, payloads, multi_points, _ = det.detectAndDecodeMulti(image_bgr)
 except cv2.error:
 ok, payloads, multi_points = False, [], None
 if ok and multi_points is not None:
 seen_payloads = {p for p, _, _ in raw_results}
 for payload, poly in zip(payloads, multi_points):
 if not payload:
 continue
 poly_pts = np.asarray(poly).reshape(-1, 2).tolist
 if payload in seen_payloads:
 continue # already reported by single-decoder pass
 raw_results.append((payload, poly_pts, _polygon_area(poly_pts)))
 seen_payloads.add(payload)

 # Largest-area first: when two cards are visible the closer one
 # (operator's actual intent) wins downstream tie-breaks. Stable so
 # equal-area results keep their decode order.
 raw_results.sort(key=lambda t: t[2], reverse=True)

 out: list[QRResult] = []
 for payload, poly_pts, _area in raw_results:
 normalised = normalize_payload(payload)
 if not normalised:
 continue
 out.append(_mk_result(normalised, poly_pts))

 return out


def temporal_vote(history: Sequence[Iterable[str]], *, min_agree: int = 3) -> str | None:
 """Return the payload that appears in at least ``min_agree`` of the last frames.

 ``history`` is a list of "payloads seen in frame N" sets. This is the
 simplest possible noise filter — we rely on a decoded QR to appear in
 multiple consecutive frames before we fire a command.
 """
 if len(history) < min_agree:
 return None
 tally: dict[str, int] = {}
 for frame_payloads in history:
 # Count each payload only once per frame to avoid a single loud frame
 # dominating the vote.
 for p in set(frame_payloads):
 tally[p] = tally.get(p, 0) + 1
 winner = max(tally.items, key=lambda kv: kv[1], default=None)
 if winner is None or winner[1] < min_agree:
 return None
 return winner[0]
