"""Project D (Hand-Gesture, sit-down posture) — additive on top of baseline.launch.py.

Launches only the gesture behaviour nodes. The arbiter, qr_detector,
command_interpreter, recorder and session_logger are expected to be
already running from baseline.launch.py (started by systemd at boot).

Use ``./scripts/scene.sh d`` to launch this on the robot.

Behaviour:
    Operator sits ~0.7 m from the camera. MediaPipe Hands (21 landmarks)
    with hold+cooldown classification. Vocabulary mirrors QR (7 verbs).

"""
from launch import LaunchDescription
from launch_ros.actions import Node


CAMERA_REMAP = ("/camera/color/image_raw", "/oak/rgb/image_raw")


def generate_launch_description():
    return LaunchDescription([
        Node(package="par_gesture", executable="gesture_detector",
             remappings=[CAMERA_REMAP],
             # Rate baked at 5 Hz (was 10 Hz) after evening
             # session: even with qr_detector killed and foxglove snap
             # stopped, MediaPipe Hands inference at 10 Hz pushed the Pi 5
             # past load 14 and triggered two spontaneous reboots. At 5 Hz
             # MediaPipe consumes ~40 % of one core (down from ~75 %) while
             # still feeling smooth (200 ms inference interval, hold_ticks=2
             # -> 400 ms hold for a confirmed gesture, 2 s cooldown between
             # gestures). Live ros2 param tuning is ineffective because the
             # timer is created at init and not recreated on rate_hz changes
             # — the launch-file bake is the only reliable lever.
             parameters=[{"cooldown_s": 2.0, "rate_hz": 5.0, "hold_ticks": 2}]),
        Node(package="par_gesture", executable="gesture_interpreter"),
    ])
