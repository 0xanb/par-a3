"""CSV + ROS event logger, used by every project to populate the report tables.

Every trial gets its own CSV under ``/workspace/logs/<trial_id>.csv``. The same
event is also published on ``/par/events`` so rosbag recording captures it.
"""
from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Telemetry:
 trial_id: str
 log_dir: str = "/workspace/logs"
 _fp: object = None
 _writer: object = None

 def __post_init__(self) -> None:
 Path(self.log_dir).mkdir(parents=True, exist_ok=True)
 path = os.path.join(self.log_dir, f"{self.trial_id}.csv")
 exists = os.path.exists(path)
 self._fp = open(path, "a", newline="")
 self._writer = csv.writer(self._fp)
 if not exists:
 self._writer.writerow(["t_unix", "event", "source", "payload", "detail"])

 def log(self, event: str, source: str = "", payload: str = "", detail: str = "") -> None:
 self._writer.writerow([time.time, event, source, payload, detail])
 self._fp.flush

 def close(self) -> None:
 if self._fp is not None:
 self._fp.close
