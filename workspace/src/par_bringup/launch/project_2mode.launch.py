"""2-mode pivot bringup — baseline + reactive (ND or VFH+) + gesture + supervisor.

Phase 1 (post ): now supports four ablation axes via launch arguments,
designed to feed the Project C trial campaign without rebuilding:

    algo : str          one of "nd_hybrid" (default) | "nd_only" | "vfh_plus"
    use_depth : bool    perception_fusion fuses depth (true) or LIDAR-only (false)
    tof_off : bool      disable H1 ToF halo (sets arbiter tof_min_m=0)
    lidar_halo_off : bool disable H2 LIDAR halo (sets lidar_stop/slow_m=0)
    detection_tier : str  one of "tight" | "default" | "wide" — scales the
                        safety + planner thresholds together (see TIER_VALUES below)

Algorithm dispatch:
 nd_hybrid → nd_planner + recovery_controller (the post- production)
 nd_only → nd_planner alone (reproduces wedge for failure-mode trials;
                Phase 0's planner-level fixes still apply, but no recovery FSM)
 vfh_plus → vfh_planner + recovery_controller (validated baseline; see ..)

Composition (no Include actions; all Node constructors visible in one place
for the rubric review):

    From baseline.launch.py:
        qr_detector, qr_command_interpreter, arbiter, recorder, session_logger
    From project_c.launch.py:
        perception_fusion (always)
        nd_planner OR vfh_planner (algo-conditional)
        recovery_controller (algo-conditional)
    From project_d.launch.py:
        gesture_detector, gesture_interpreter
    NEW for this launch:
        supervisor (cold-boot self-validate + 360 announce + IDLE latch)

Behaviour nodes already gate on /par/active_mode (TRANSIENT_LOCAL latched
on the supervisor side), so launching them all simultaneously is safe: the
supervisor publishes IDLE on entry and every behaviour stays silent until
the operator runs scripts/scene.sh a or scripts/scene.sh d.

Detection tier mapping (all values in metres):

    Tier      safety_dist_m  obstacle_threshold_m  lidar_stop_m  lidar_slow_m
    tight       0.20             0.30                 0.12           0.25
    default     0.30             0.45                 0.18           0.35
    wide        0.40             0.60                 0.25           0.50

Each individual threshold argument (safety_dist_m, obstacle_threshold_m,
lidar_stop_m, lidar_slow_m) defaults to the sentinel "auto", meaning "use
the tier-derived value". Pass any of them explicitly (e.g. `lidar_stop_m:=0.10`)
to override the tier for that one parameter only.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_REMAP = ("/camera/color/image_raw", "/oak/rgb/image_raw")

TIER_VALUES = {
    "tight":   {"safety_dist_m": 0.20, "obstacle_threshold_m": 0.30,
                "lidar_stop_m": 0.12, "lidar_slow_m": 0.25},
    "default": {"safety_dist_m": 0.30, "obstacle_threshold_m": 0.45,
                "lidar_stop_m": 0.18, "lidar_slow_m": 0.35},
    "wide":    {"safety_dist_m": 0.40, "obstacle_threshold_m": 0.60,
                "lidar_stop_m": 0.25, "lidar_slow_m": 0.50},
}

VALID_ALGOS = ("nd_hybrid", "nd_only", "vfh_plus")


def _resolve_threshold(arg_value: str, tier_value: float) -> float:
    """Sentinel "auto" → tier default; otherwise float(arg_value)."""
    if arg_value == "auto":
        return tier_value
    return float(arg_value)


def _bool_arg(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _build_nodes(context, *args, **kwargs):
    # Resolve tier
    tier = LaunchConfiguration("detection_tier").perform(context)
    if tier not in TIER_VALUES:
        raise ValueError(
            f"detection_tier must be one of {list(TIER_VALUES.keys())}, got {tier!r}"
        )
    t = TIER_VALUES[tier]

    # Resolve algorithm
    algo = LaunchConfiguration("algo").perform(context)
    if algo not in VALID_ALGOS:
        raise ValueError(f"algo must be one of {VALID_ALGOS}, got {algo!r}")

    # Resolve thresholds (sentinel "auto" → tier default)
    safety_dist_m = _resolve_threshold(
        LaunchConfiguration("safety_dist_m").perform(context), t["safety_dist_m"]
    )
    obstacle_threshold_m = _resolve_threshold(
        LaunchConfiguration("obstacle_threshold_m").perform(context),
        t["obstacle_threshold_m"],
    )
    lidar_stop_m = _resolve_threshold(
        LaunchConfiguration("lidar_stop_m").perform(context), t["lidar_stop_m"]
    )
    lidar_slow_m = _resolve_threshold(
        LaunchConfiguration("lidar_slow_m").perform(context), t["lidar_slow_m"]
    )
    chassis_half_width_m = float(
        LaunchConfiguration("chassis_half_width_m").perform(context)
    )

    # Safety-layer overrides (ablation axes)
    tof_off = _bool_arg(LaunchConfiguration("tof_off").perform(context))
    lidar_halo_off = _bool_arg(
        LaunchConfiguration("lidar_halo_off").perform(context)
    )
    if tof_off:
        tof_min_m = 0.0
    else:
        tof_min_m = float(LaunchConfiguration("tof_min_m").perform(context))
    if lidar_halo_off:
        lidar_stop_m = 0.0
        lidar_slow_m = 0.0

    v_max = LaunchConfiguration("v_max")
    w_max = LaunchConfiguration("w_max")
    trial_id = LaunchConfiguration("trial_id")
    disable_halos = LaunchConfiguration("disable_proximity_halos")
    use_depth = LaunchConfiguration("use_depth")
    validate_timeout_s = LaunchConfiguration("validate_timeout_s")
    spin_rate_rad_s = LaunchConfiguration("spin_rate_rad_s")
    announce_enabled = _bool_arg(LaunchConfiguration("announce_enabled").perform(context))

    # Always-present nodes
    nodes = [
        Node(package="par_qr_nav", executable="qr_detector", name="qr_detector",
             remappings=[CAMERA_REMAP]),
        Node(package="par_qr_nav", executable="command_interpreter",
             name="qr_command_interpreter"),
        Node(package="par_arbiter", executable="arbiter", name="arbiter",
             parameters=[{
                 "v_max": v_max,
                 "w_max": w_max,
                 "disable_proximity_halos": disable_halos,
                 "tof_min_m": tof_min_m,
                 "lidar_stop_m": lidar_stop_m,
                 "lidar_slow_m": lidar_slow_m,
             }]),
        Node(package="par_eval", executable="recorder", name="recorder",
             parameters=[{"trial_id": trial_id}]),
        Node(package="par_eval", executable="session_logger", name="session_logger"),
        Node(package="par_eval", executable="snapshotter", name="snapshotter",
             remappings=[CAMERA_REMAP]),
        Node(package="par_anomaly", executable="anomaly_detector",
             name="anomaly_detector"),
        Node(package="par_reactive_nav", executable="perception_fusion",
             parameters=[{"use_depth": use_depth},
                         {"use_lidar": LaunchConfiguration("use_lidar")}]),
    ]

    # Algorithm-conditional planner
    if algo in ("nd_hybrid", "nd_only"):
        nodes.append(Node(
            package="par_reactive_nav", executable="nd_planner", name="nd_planner",
            parameters=[{
                "safety_dist_m": safety_dist_m,
                "chassis_half_width_m": chassis_half_width_m,
            }],
        ))
    elif algo == "vfh_plus":
        nodes.append(Node(
            package="par_reactive_nav", executable="vfh_planner", name="vfh_planner",
            parameters=[{
                "obstacle_threshold_m": obstacle_threshold_m,
                "chassis_half_width_m": chassis_half_width_m,
            }],
        ))

    # Recovery controller — only for harness-equipped variants. nd_only deliberately
    # omits this to reproduce the wedge for failure-mode trials.
    #
    # : recovery_trigger_hold_s defaults to 3.0 s for vfh_plus
    # (vs 0.3 s for nd_hybrid). vfh_planner publishes a transient DEAD_END at
    # cold start because the initial polar histogram has not yet integrated
    # depth + LIDAR; at 0.3 s recovery fires before perception settles, the
    # arbiter clamps the reverse intent, and the planner wedges . Three
    # seconds gives perception time to converge before recovery arms. Override
    # via `recovery_trigger_hold_s:=0.3` for back-compat trials.
    if algo in ("nd_hybrid", "vfh_plus"):
        recovery_hold_arg = LaunchConfiguration(
            "recovery_trigger_hold_s"
        ).perform(context)
        if recovery_hold_arg == "auto":
            recovery_trigger_hold_s = 3.0 if algo == "vfh_plus" else 0.3
        else:
            recovery_trigger_hold_s = float(recovery_hold_arg)
        nodes.append(Node(
            package="par_reactive_nav", executable="recovery_controller",
            name="recovery_controller",
            parameters=[{"trigger_hold_s": recovery_trigger_hold_s}],
        ))

    # Gesture stack — same as project_d.launch.py rate/cooldown/hold.
    nodes.extend([
        Node(package="par_gesture", executable="gesture_detector",
             remappings=[CAMERA_REMAP],
             parameters=[{"cooldown_s": 2.0, "rate_hz": 5.0, "hold_seconds": 0.4}]),
        Node(package="par_gesture", executable="gesture_interpreter"),
        Node(package="par_supervisor", executable="supervisor", name="supervisor",
             parameters=[{
                 "validate_timeout_s": validate_timeout_s,
                 "spin_rate_rad_s": spin_rate_rad_s,
                 "announce_enabled": announce_enabled,
             }]),
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ---- baseline args ------------------------------------------------
        DeclareLaunchArgument("v_max", default_value="0.10",
                              description="Arbiter linear speed cap (m/s). "
                                          "Cautious tier default."),
        DeclareLaunchArgument("w_max", default_value="1.20",
                              description="Arbiter angular speed cap (rad/s)."),
        DeclareLaunchArgument("trial_id", default_value="2mode",
                              description="Trial identifier propagated to recorder."),
        DeclareLaunchArgument("disable_proximity_halos", default_value="false",
                              description="Bench mode: disable H1 ToF + H2 LIDAR "
                                          "halos at the arbiter."),
        # ---- detection-tier ----------------------------------------------
        DeclareLaunchArgument(
            "detection_tier", default_value="default",
            description="Threshold preset: 'tight' (0.20/0.30/0.12/0.25), "
                        "'default' (0.30/0.45/0.18/0.35), 'wide' "
                        "(0.40/0.60/0.25/0.50). Each individual threshold can "
                        "still be overridden via its own arg below.",
        ),
        DeclareLaunchArgument("safety_dist_m", default_value="auto",
                              description="ND classifier free-region threshold. "
                                          "'auto' = use detection_tier value."),
        DeclareLaunchArgument("obstacle_threshold_m", default_value="auto",
                              description="VFH+ blocked-bin threshold. "
                                          "'auto' = use detection_tier value."),
        DeclareLaunchArgument("lidar_stop_m", default_value="auto",
                              description="H2 LIDAR halo hard-stop distance (m). "
                                          "'auto' = use detection_tier value."),
        DeclareLaunchArgument("lidar_slow_m", default_value="auto",
                              description="H2 LIDAR halo soft-slow distance (m). "
                                          "'auto' = use detection_tier value."),
        DeclareLaunchArgument("chassis_half_width_m", default_value="0.165",
                              description="Half the chassis width (m), used by "
                                          "Borenstein-Koren angular obstacle "
                                          "inflation in nd_core / vfh_core."),
        DeclareLaunchArgument("tof_min_m", default_value="0.12",
                              description="H1 ToF halo distance (m). Override "
                                          "via tof_off:=true to disable."),
        DeclareLaunchArgument("tof_off", default_value="false",
                              description="Disable H1 ToF halo for ablation "
                                          "(controlled v=0.05 m/s safety trial)."),
        DeclareLaunchArgument("lidar_halo_off", default_value="false",
                              description="Disable H2 LIDAR halo (rare ablation)."),
        # ---- algorithm dispatch ------------------------------------------
        DeclareLaunchArgument(
            "algo", default_value="nd_hybrid",
            description="Reactive algorithm: 'nd_hybrid' (ND classifier + "
                        "recovery_controller), 'nd_only' "
                        "(ND classifier without recovery FSM, reproduces the "
                        "wedge), 'vfh_plus' (validated baseline + recovery).",
        ),
        # ---- ND stack args ------------------------------------------------
        DeclareLaunchArgument("use_depth", default_value="true",
                              description="perception_fusion fuses OAK depth "
                                          "into the polar histogram."),
        DeclareLaunchArgument("use_lidar", default_value="true",
                              description="perception_fusion fuses /scan into the "
                                          "polar histogram. Set false for the "
                                          "depth-only ablation cell."),
        DeclareLaunchArgument(
            "recovery_trigger_hold_s", default_value="auto",
            description="recovery_controller trigger_hold_s (s). 'auto' = 3.0 for "
                        "vfh_plus, 0.3 for nd_hybrid (cold-start "
                        "wedge mitigation). Override with a numeric value.",
        ),
        # ---- supervisor args ----------------------------------------------
        DeclareLaunchArgument("validate_timeout_s", default_value="60.0",
                              description="supervisor cold-boot self-validate window (s)."),
        DeclareLaunchArgument("spin_rate_rad_s", default_value="0.3",
                              description="supervisor 360 announce angular rate "
                                          "(rad/s)."),
        DeclareLaunchArgument("announce_enabled", default_value="true",
                              description="Enable supervisor cold-boot 360° "
                                          "readiness rotation. True for "
                                          "production / demo (operator visible "
                                          "ready signal); set false for trial "
                                          "harness to avoid auto-yaw between "
                                          "trial runs."),

        # Build the node list at launch-execute time so we can resolve tier +
        # algo conditionals against the actual LaunchConfiguration values.
        OpaqueFunction(function=_build_nodes),
    ])
