"""Pure-function tests for par_core.snapshot.

Loaded via file path to avoid the ``par_core/__init__.py`` ROS imports.
"""
import importlib.util
import pathlib
import time

_HERE = pathlib.Path(__file__).resolve.parent
_MOD_PATH = _HERE.parent / "par_core" / "snapshot.py"
_spec = importlib.util.spec_from_file_location("snapshot_under_test", _MOD_PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
make_capture_path = _mod.make_capture_path
trim_to_cap = _mod.trim_to_cap


def test_capture_filename_encodes_source_ts_label(tmp_path) -> None:
 p = make_capture_path("qr", "STOP", ts=1700000000.123, directory=tmp_path)
 assert p.parent == tmp_path
 # 13-digit zero-padded ms (1.7e12 millis = 13 digits already).
 assert p.name == "qr_1700000000123_STOP.jpg"


def test_capture_filename_sanitises_label(tmp_path) -> None:
 p = make_capture_path("gesture", "open palm/v2", ts=1.0, directory=tmp_path)
 # Slashes and spaces become underscores so the filename remains a single
 # path segment.
 assert "/" not in p.name
 assert " " not in p.name


def test_filename_sort_matches_time_sort(tmp_path) -> None:
 """Lexical sort must match ascending ts so external tooling (find, ls)
 can iterate captures in chronological order without parsing."""
 paths = [make_capture_path("qr", "X", ts=t, directory=tmp_path)
 for t in (1.0, 2.5, 100.0, 99.0)]
 name_sorted = sorted(p.name for p in paths)
 ts_sorted = [p.name for p in sorted(paths, key=lambda p: int(p.name.split("_")[1]))]
 assert name_sorted == ts_sorted


def test_trim_drops_oldest_files(tmp_path) -> None:
 """Write 6 files; trim to 4 should delete the 2 oldest by mtime."""
 files = []
 for i in range(6):
 p = tmp_path / f"qr_{i:08d}_X.jpg"
 p.write_bytes(b"x")
 # Set mtime so the order is unambiguous on fast filesystems.
 os.utime(p, (1000.0 + i, 1000.0 + i))
 files.append(p)
 removed = trim_to_cap(tmp_path, max_files=4)
 assert removed == 2
 surviving = sorted(p.name for p in tmp_path.glob("*.jpg"))
 # Files 0 and 1 are oldest; should be gone.
 assert "qr_00000000_X.jpg" not in surviving
 assert "qr_00000001_X.jpg" not in surviving
 # Files 2-5 should remain.
 assert len(surviving) == 4


def test_trim_no_op_when_under_cap(tmp_path) -> None:
 for i in range(3):
 (tmp_path / f"x_{i}.jpg").write_bytes(b"x")
 removed = trim_to_cap(tmp_path, max_files=10)
 assert removed == 0
 assert len(list(tmp_path.glob("*.jpg"))) == 3


def test_trim_handles_missing_directory(tmp_path) -> None:
 assert trim_to_cap(tmp_path / "does_not_exist") == 0


# Standard-lib import for os.utime above. Placed at the bottom because the
# file-loader-style header up top must run first.
import os # noqa: E402
