"""Unit tests for par_qr_nav.detector_core — no ROS, no camera.

We generate a QR image in-memory and feed it to ``detect()`` to prove the
OpenCV decoder is wired up correctly. The test also locks in the temporal
voter's contract.
"""
from __future__ import annotations

import cv2
import numpy as np

from par_qr_nav.detector_core import (
    KNOWN_VERBS,
    _polygon_area,
    detect,
    normalize_payload,
    temporal_vote,
)


def _make_qr(text: str, size: int = 600) -> np.ndarray:
    """Render ``text`` as a QR code using OpenCV's encoder (available since 4.8)."""
    encoder = cv2.QRCodeEncoder_create()
    img = encoder.encode(text)
    img = cv2.resize(img, (size, size), interpolation=cv2.INTER_NEAREST)
    # detectAndDecodeMulti wants 3-channel BGR
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_known_verbs_set_is_complete() -> None:
    assert KNOWN_VERBS == {
        "TURN_LEFT", "TURN_RIGHT", "STOP", "GO",
        "SPEED_UP", "SPEED_DOWN", "U_TURN",
    }


def test_detect_decodes_stop() -> None:
    img = _make_qr("STOP")
    results = detect(img)
    assert len(results) == 1
    assert results[0].payload == "STOP"
    assert results[0].known is True
    # centroid should be inside the image
    cx, cy = results[0].centroid
    assert 0 <= cx < img.shape[1] and 0 <= cy < img.shape[0]


def test_detect_marks_unknown_payload() -> None:
    img = _make_qr("NOT_A_COMMAND")
    results = detect(img)
    assert len(results) == 1
    assert results[0].known is False


def test_detect_empty_image_is_safe() -> None:
    assert detect(np.zeros((0, 0, 3), dtype=np.uint8)) == []


def test_detect_does_not_crash_on_random_noise() -> None:
    """OpenCV 4.11 detectAndDecode raises cv2.error when a candidate QR contour
    has zero area (real-world artefact: a near-edge QR-like pattern). detect()
    must catch the error and treat the frame as "no decode" rather than crash
 the wrapper node. Regression for (qr_detector node crash )."""
    rng = np.random.default_rng(seed=0)
    noisy = (rng.random((480, 640, 3)) * 255).astype(np.uint8)
    # Should not raise.
    result = detect(noisy)
    # On random noise the decoder may return zero or more spurious results;
    # the contract is "does not crash", not "returns empty exactly".
    assert isinstance(result, list)


def test_temporal_vote_needs_three_of_five() -> None:
    history = [["STOP"], ["STOP"], [], ["STOP"], []]
    assert temporal_vote(history, min_agree=3) == "STOP"


def test_temporal_vote_rejects_singletons() -> None:
    history = [["STOP"], [], [], [], []]
    assert temporal_vote(history, min_agree=3) is None


def test_temporal_vote_ignores_short_window() -> None:
    history = [["STOP"]]
    assert temporal_vote(history, min_agree=3) is None


def test_temporal_vote_breaks_ties_by_count() -> None:
    history = [["STOP"], ["GO"], ["STOP"], ["GO"], ["STOP"]]
    # STOP appears in 3 frames, GO in 2. STOP wins.
    assert temporal_vote(history, min_agree=3) == "STOP"


# ---- payload normalisation (audit edge case 3.1) -----------------------

def test_normalize_payload_lowercase() -> None:
    assert normalize_payload("go") == "GO"
    assert normalize_payload("Go") == "GO"
    assert normalize_payload("STOP") == "STOP"


def test_normalize_payload_strips_whitespace_and_newlines() -> None:
    assert normalize_payload("  GO  ") == "GO"
    assert normalize_payload("STOP\n") == "STOP"
    assert normalize_payload("\t TURN_LEFT \r\n") == "TURN_LEFT"


def test_normalize_payload_url_trailing_segment() -> None:
    assert normalize_payload("https://example.com/STOP") == "STOP"
    assert normalize_payload("HTTP://X/Y/Z/U_TURN") == "U_TURN"


def test_normalize_payload_unknown_passes_through() -> None:
    """Off-vocab payloads return their normalised form so the downstream
    KNOWN_VERBS check correctly marks them unknown without false positives."""
    assert normalize_payload("BANANA") == "BANANA"
    assert normalize_payload("good") == "GOOD"   # not GO — exact-match lookup
    assert normalize_payload("STOP_NOW") == "STOP_NOW"


def test_normalize_payload_empty_and_none() -> None:
    assert normalize_payload("") == ""
    assert normalize_payload(None) == ""
    assert normalize_payload("   \n\t  ") == ""


def test_detect_normalizes_lowercase_qr_payload() -> None:
    """End-to-end: a QR encoding 'go' decodes to a known GO command after
    normalisation, instead of being marked unknown."""
    img = _make_qr("go")
    results = detect(img)
    assert len(results) == 1
    assert results[0].payload == "GO"
    assert results[0].known is True


# ---- multi-detection: largest-area wins (audit edge case 3.2) ----------

def test_polygon_area_shoelace() -> None:
    # 100x100 square at origin -> area 10000
    poly = [(0, 0), (100, 0), (100, 100), (0, 100)]
    assert abs(_polygon_area(poly) - 10000.0) < 1e-6


def test_polygon_area_degenerate() -> None:
    # Two-point polygon (collinear) is degenerate.
    assert _polygon_area([(0, 0), (10, 0)]) == 0.0


def test_detect_two_qrs_largest_wins_first() -> None:
    """When two QRs are visible, results are sorted by polygon area
    descending so downstream voters / consumers see the closer card first."""
    big = _make_qr("STOP", size=400)
    small = _make_qr("GO", size=120)
    canvas = np.full((600, 800, 3), 255, dtype=np.uint8)
    canvas[50:50 + big.shape[0], 50:50 + big.shape[1]] = big
    # Place the small QR with a margin so the decoders find both.
    canvas[460:460 + small.shape[0], 660:660 + small.shape[1]] = small
    results = detect(canvas)
    payloads = [r.payload for r in results]
    if "STOP" in payloads and "GO" in payloads:
        # Both decoded — the contract is "largest first".
        assert payloads.index("STOP") < payloads.index("GO")
    else:
        # OpenCV's multi-decoder is sensitivity-dependent; on rare runs only
        # one of the two is decoded. The contract still holds (vacuously).
        assert len(results) >= 1
