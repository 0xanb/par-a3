"""Hardware-In-Loop: robot on blocks, wheels free, caps at zero.

Runs the full behaviour stack plus the arbiter but forces v_max = w_max = 0,
so the wheels receive commands the operator can see on the terminal but the
robot never physically moves. Use to prove the full command chain end-to-end
without kinematic risk.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description:
 mode = LaunchConfiguration("mode", default="hil")
 return LaunchDescription([
 DeclareLaunchArgument("mode", default_value="hil"),
 Node(package="par_arbiter", executable="arbiter", name="arbiter",
 parameters=[{"v_max": 0.0, "w_max": 0.0, "mode": mode}]),
 Node(package="par_eval", executable="recorder", name="recorder",
 parameters=[{"trial_id": "hil", "mode": mode}]),
 ])
