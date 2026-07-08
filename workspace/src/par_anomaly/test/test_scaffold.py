"""Smoke test — par_anomaly imports cleanly without ROS at import time."""
from __future__ import annotations


def test_detectors_imports_without_ros() -> None:
    """Importing par_anomaly.detectors must NOT require ROS / rclpy.

    Detectors are pure-function predicates; the ROS wrapper lives in
    anomaly_detector.py with lazy imports. Host tests must be runnable
    outside the dev container so the campaign post-processing scripts
    can call into the predicates directly.
    """
    import par_anomaly.detectors  # noqa: F401


def test_package_metadata() -> None:
    import par_anomaly

    assert par_anomaly is not None
