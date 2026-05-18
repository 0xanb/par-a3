"""Tests for par_eval.session_logger formatters.

Only the pure-function helpers are exercised; the SessionLogger ROS node is
covered by manual on-robot inspection of ``log.txt``.
"""
from par_eval.session_logger import (
 format_detect_line,
 format_event_line,
 format_intent_line,
 format_log_line,
 format_mode_line,
)


def test_intent_line_columnar -> None:
 line = format_intent_line(
 t_s=1700000000.234,
 source="qr",
 label="STOP",
 priority=85,
 v=0.0,
 w=0.0,
 )
 assert "intent" in line
 assert "qr" in line
 assert "STOP" in line
 assert "priority=85" in line
 assert "v=0.00" in line


def test_detect_line_carries_confidence -> None:
 line = format_detect_line(
 t_s=1700000000.300, source="qr", payload="GO", confidence=0.95,
 )
 assert "confidence=0.95" in line
 assert "detect" in line


def test_event_line_optional_detail -> None:
 with_detail = format_event_line(t_s=1.0, source="reactive",
 event="stale_perception", detail="0.8s")
 assert "detail=\"0.8s\"" in with_detail
 no_detail = format_event_line(t_s=1.0, source="qr", event="recovering")
 assert "detail=" not in no_detail


def test_mode_line_no_source_provenance -> None:
 line = format_mode_line(t_s=1.0, mode="A", reason="boot")
 assert "mode" in line
 assert "reason=boot" in line
 # Mode rows use "-" in the source column.
 assert " - " in line or " - " in line


def test_log_line_has_level_and_name -> None:
 line = format_log_line(
 t_s=1700000000.500, level="WARN", name="vfh_planner",
 msg="stale_perception",
 )
 assert "WARN" in line
 assert "vfh_planner" in line
 assert "stale_perception" in line


def test_lines_are_grep_friendly -> None:
 """A common operator workflow is ``grep ERROR log.txt``. Confirm the
 level token is on a word boundary so grep with -w works."""
 line = format_log_line(t_s=1.0, level="ERROR", name="x", msg="boom")
 parts = line.split
 assert "ERROR" in parts
