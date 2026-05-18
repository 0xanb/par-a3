"""Frame-snapshot helper used by every detector to save a JPEG when it
commits a detection.

Each call writes a file under ``<session_dir>/captures/<source>_<unix_ts>_<label>.jpg``.
The directory is capped at ``MAX_FILES`` via FIFO ring buffer to keep the
SD card healthy during long sessions.

Importable without ROS so it can be exercised in host tests.
"""
from __future__ import annotations

import datetime as _dt
import os
import pathlib

# Capture cap so a long-running session does not fill the SD card. ~120 KB
# per JPEG × 500 ≈ 60 MB ceiling.
MAX_FILES: int = 500

# Allowed JPEG quality range. 85 balances readability vs file size.
DEFAULT_JPEG_QUALITY: int = 85


def session_dir -> pathlib.Path:
 """Return the per-session directory. Honours ``PAR_A3_SESSION_DIR`` if
 set, otherwise computes ``~/par-a3-logs/session_<YYYYMMDD_HHMM>`` once
 per process. Subsequent calls return the same path."""
 cached = getattr(session_dir, "_cached", None)
 if cached is not None:
 return cached
 env = os.environ.get("PAR_A3_SESSION_DIR")
 if env:
 path = pathlib.Path(env).expanduser
 else:
 stamp = _dt.datetime.now.strftime("%Y%m%d_%H%M")
 path = pathlib.Path("~/par-a3-logs").expanduser / f"session_{stamp}"
 path.mkdir(parents=True, exist_ok=True)
 setattr(session_dir, "_cached", path)
 return path


def captures_dir -> pathlib.Path:
 out = session_dir / "captures"
 out.mkdir(parents=True, exist_ok=True)
 return out


def make_capture_path(source: str, label: str, *, ts: float | None = None,
 directory: pathlib.Path | str | None = None) -> pathlib.Path:
 """Pure-function helper used by the snapshot writer and tested directly.

 Filename layout: ``<source>_<unix_ts_millis>_<label>.jpg`` so a sort by
 name is also a sort by time.
 """
 if ts is None:
 ts = _dt.datetime.now.timestamp
 safe_label = "".join(c if (c.isalnum or c in "-_") else "_" for c in label)
 # Zero-pad ms timestamp to 13 digits so a lexical sort of the filenames
 # matches the chronological sort. 13 digits covers any timestamp up to
 # year 5138, which is past anyone's hardware lifetime.
 fname = f"{source}_{int(ts * 1000):013d}_{safe_label}.jpg"
 base = pathlib.Path(directory) if directory is not None else captures_dir
 return base / fname


def trim_to_cap(directory: pathlib.Path | str, *, max_files: int = MAX_FILES) -> int:
 """Delete oldest *.jpg files until the directory has at most ``max_files``.
 Returns how many files were removed. Pure-function-friendly: takes only
 a directory and a cap, and uses mtime ordering.
 """
 base = pathlib.Path(directory)
 if not base.exists:
 return 0
 jpgs = sorted(base.glob("*.jpg"), key=lambda p: p.stat.st_mtime)
 excess = max(0, len(jpgs) - max_files)
 for path in jpgs[:excess]:
 try:
 path.unlink
 except OSError:
 pass
 return excess


def save_snapshot(
 frame_bgr,
 source: str,
 label: str,
 *,
 quality: int = DEFAULT_JPEG_QUALITY,
 directory: pathlib.Path | str | None = None,
) -> pathlib.Path | None:
 """Encode ``frame_bgr`` as JPEG and save with the canonical filename.

 Returns the path on success or None if the frame could not be saved
 (cv2 missing, frame is None, write failure). Lazy-imports cv2 because
 par_core must be importable in test environments without OpenCV.
 """
 if frame_bgr is None:
 return None
 try:
 import cv2 # noqa: PLC0415
 except ImportError:
 return None
 path = make_capture_path(source, label, directory=directory)
 ok = cv2.imwrite(str(path), frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
 if not ok:
 return None
 trim_to_cap(path.parent)
 return path
