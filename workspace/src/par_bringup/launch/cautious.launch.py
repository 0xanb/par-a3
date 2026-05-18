"""First-ground-run config: tethered, padded arena, very low caps.

v_max 0.10 m/s, w_max 0.50 rad/s. Only after two clean cautious runs do we
bump to `normal` (v_max 0.20, w_max 1.0) and only after two clean normals do
we enable `demo` (v_max 0.40, w_max 1.5). Never skip rungs.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description:
 mode = LaunchConfiguration("mode", default="cautious")
 return LaunchDescription([
 DeclareLaunchArgument("mode", default_value="cautious"),
 Node(package="par_arbiter", executable="arbiter", name="arbiter",
 parameters=[{"v_max": 0.10, "w_max": 0.50, "mode": mode}]),
 Node(package="par_eval", executable="recorder", name="recorder",
 parameters=[{"trial_id": "cautious", "mode": mode}]),
 ])
