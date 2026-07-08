"""Always-on baseline stack for systemd boot.

Brings up only the nodes that must run continuously regardless of which
scene is being demonstrated:

    arbiter                  — priority fusion + SafetyLayer (LIDAR + ToF halos)
    qr_detector              — scene A perception (also the operator's mode-
                                indicator card channel for telemetry)
    qr_command_interpreter   — QR verb -> CommandIntent
    recorder                 — rosbag of /par/* topics for trial analysis
    session_logger           — text log of intents, detections, mode markers

Scene B/C/D nodes are NOT started here. The operator launches them
on demand via ``./scripts/scene.sh <b|c|d>`` (which SSH-launches the
relevant ``project_<x>.launch.py``). Each project launch layers
additively onto this baseline — the arbiter stays running and just
starts seeing intents from the new source.

Why we do it this way (vs the earlier all-in-one mode-driven runtime):

  The mode-driven design (see workspace/src/-archived/mode-driven-runtime/
 + -revised) launched every behaviour
  node simultaneously and gated their publishes by /par/active_mode. On
  the ROSbot 3 PRO's 4-core CPU this hit load averages of 15+ even at
  10 fps camera input, with depthai contention causing visible Foxglove
  lag. The hybrid baseline + on-demand launch model recovers ~70% of
  that load while preserving the demo flow.

Launch arguments (all optional, normally set by par_a3_runtime.sh env):
    v_max                       Arbiter linear speed cap (m/s).
    w_max                       Arbiter angular speed cap (rad/s).
    trial_id                    Recorder trial id.
    disable_proximity_halos     Bench mode: skip H1 ToF + H2 LIDAR halos.
    lidar_stop_m                LIDAR hard-stop distance (m).
    lidar_slow_m                LIDAR start-of-slowdown distance (m).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_REMAP = ("/camera/color/image_raw", "/oak/rgb/image_raw")


def generate_launch_description():
    v_max = LaunchConfiguration("v_max")
    w_max = LaunchConfiguration("w_max")
    trial_id = LaunchConfiguration("trial_id")
    disable_halos = LaunchConfiguration("disable_proximity_halos")
    lidar_stop_m = LaunchConfiguration("lidar_stop_m")
    lidar_slow_m = LaunchConfiguration("lidar_slow_m")

    return LaunchDescription([
        DeclareLaunchArgument("v_max", default_value="0.20",
                              description="Arbiter linear speed cap (m/s)."),
        DeclareLaunchArgument("w_max", default_value="1.20",
                              description="Arbiter angular speed cap (rad/s)."),
        DeclareLaunchArgument("trial_id", default_value="baseline"),
        DeclareLaunchArgument("disable_proximity_halos", default_value="false",
                              description="Bench mode: skip H1 ToF + H2 LIDAR halos."),
        DeclareLaunchArgument("lidar_stop_m", default_value="0.20",
                              description="Forward-cone LIDAR distance (m) at which the SafetyLayer hard-stops the robot."),
        DeclareLaunchArgument("lidar_slow_m", default_value="0.40",
                              description="Forward-cone LIDAR distance (m) at which the SafetyLayer starts linearly scaling speed down."),
        Node(package="par_qr_nav", executable="qr_detector", name="qr_detector",
             remappings=[CAMERA_REMAP]),
        Node(package="par_qr_nav", executable="command_interpreter", name="qr_command_interpreter"),
        Node(package="par_arbiter", executable="arbiter", name="arbiter",
             parameters=[{
                 "v_max": v_max,
                 "w_max": w_max,
                 "disable_proximity_halos": disable_halos,
                 "lidar_stop_m": lidar_stop_m,
                 "lidar_slow_m": lidar_slow_m,
             }]),
        Node(package="par_eval", executable="recorder", name="recorder",
             parameters=[{"trial_id": trial_id}]),
        Node(package="par_eval", executable="session_logger", name="session_logger"),
    ])
