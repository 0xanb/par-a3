"""Project A (QR Navigation) — additive on top of baseline.launch.py.

Note: baseline.launch.py already starts qr_detector + qr_command_interpreter
(QR is the always-on baseline channel because it doubles as scene A.the operator's mode-indicator telemetry channel). So this launch file is
intentionally empty when baseline is already running — it exists for
parity with project_b/c/d and so ``./scripts/scene.sh a`` has something
to invoke even though the answer is "do nothing".

If you really need to bring up the QR pipeline standalone (no baseline),
use baseline.launch.py instead.
"""
from launch import LaunchDescription


def generate_launch_description:
 # Empty: baseline.launch.py already starts qr_detector +
 # qr_command_interpreter + arbiter + recorder + session_logger.
 return LaunchDescription([])
