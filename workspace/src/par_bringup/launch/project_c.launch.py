"""Project C (Reactive ND) — Nearness Diagram, primary as of.

Launches only the reactive-nav behaviour nodes. The arbiter, qr_detector,
command_interpreter, recorder and session_logger are expected to be
already running from baseline.launch.py (started by systemd at boot).

Use ``./scripts/scene.sh c`` to launch this on the robot. ``scene.sh c``
also flips the OAK depthai snap pipeline to RGBD so depth fusion is
available; other scenes leave the snap in RGB-only mode for performance
.

The original VFH+ planner + recovery_controller stack is preserved as
``project_c0.launch.py`` and reachable via ``./scripts/scene.sh c0`` for
A/B comparison. Both launches share `perception_fusion` and the same
arbiter publication contract; they differ only in which planner consumes
the polar histogram.

Behaviour:
 perception_fusion — LIDAR polar histogram, optionally fused with depth
 (use_depth:=true).
 nd_planner — Minguez & Montano 2004 Nearness Diagram. Five-state
 classifier (HSGR / HSWR / HSNR / LS1 / LS2) plus
 LS2_BACKUP for geometric dead-ends. The LS2_BACKUP
 state subsumes the recovery_controller used by
 VFH+: this launch deliberately omits that node.

Launch arguments:
 use_depth Default true here (Project C is the depth scene). Set to
 false to A/B-test LIDAR-only fusion.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description:
 use_depth = LaunchConfiguration("use_depth")
 return LaunchDescription([
 DeclareLaunchArgument(
 "use_depth", default_value="true",
 description="Whether perception_fusion fuses depth into the "
 "polar histogram. Default true for Project C; the "
 "node-level default is false (other scenes do not "
 "need depth)."),
 Node(package="par_reactive_nav", executable="perception_fusion",
 parameters=[{"use_depth": use_depth}]),
 Node(package="par_reactive_nav", executable="nd_planner"),
 ])
