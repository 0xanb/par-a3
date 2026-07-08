"""Project C0 (Reactive VFH+ — backup baseline, preserved ).

This is the original VFH+ stack frozen as a fallback after scene C was
re-implemented with the Nearness Diagram (Minguez & Montano 2004). Both
launches are operationally identical (same perception_fusion, same arbiter
contract, same priority); they differ only in which planner consumes the
polar histogram and how dead-end recovery is shaped.

Use this scene when you want the well-tested VFH+ + recovery_controller
behaviour, e.g. for the demo-day comparison run or to A/B-test against ND.

Use ``./scripts/scene.sh c0`` to launch on the robot.

Behaviour:
    perception_fusion           — LIDAR polar histogram, optionally fused
                                   with depth (use_depth:=true).
    vfh_planner                 — picks best forward valley using cost
                                   function: forward_bias + smoothness +
                                   depth_bonus. Stuck-watchdog forces
                                   DEAD_END after N near-zero ticks.
    recovery_controller         — separate FSM that spin/reverses out of
                                   dead-ends with explicit cooldown.

Launch arguments:
    use_depth   Default true. Set to false to A/B-test LIDAR-only fusion.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        Node(package="par_reactive_nav", executable="vfh_planner"),
        Node(package="par_reactive_nav", executable="recovery_controller"),
    ])
