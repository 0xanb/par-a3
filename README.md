# par-a3

A multi-modal autonomous-navigation stack for the **Husarion ROSbot 3 PRO**, built in ROS 2 Jazzy. The robot combines reactive obstacle avoidance, a QR-code operator-command channel, and a hand-gesture command channel beneath a single priority-with-freshness arbiter and a defense-in-depth safety layer.

## What this project addresses

The assignment brief asked for one or more autonomous behaviors on the ROSbot 3 PRO platform: QR-code command navigation, traffic-light perception, and reactive obstacle avoidance, plus an optional creativity extension. The constraints were tight: a single-board compute envelope (Raspberry Pi 5, ARM64 quad-core Cortex-A76 at 2.4 GHz, 8 GB RAM) that excludes map-based planning and learned policies, a sensor envelope with documented blind zones below the LIDAR scan plane, and a hard 10 Hz control-loop budget alongside the Husarion driver.

The system in this repository goes beyond the brief by integrating three behaviors that share one arbitration contract:

- **Reactive obstacle avoidance** — `nd_hybrid`, a Nearness Diagram named-situation classifier over a VFH+ polar histogram, augmented with a depth-projected channel for sub-LIDAR obstacles, a recovery finite state machine, and an IMU-plus-stall anomaly layer.
- **QR-code operator commands** — single-code primary decoder with multi-code fallback, two debouncing stages (3-of-N temporal voter and a TTL gate), priority-aware two-card resolver, and a Mode-A whitelist that restricts the channel to pause-and-resume during reactive driving.
- **Hand-gesture operator commands (extension)** — MediaPipe Hands landmark extraction at 5 Hz in Mode D, a rule-based finger-extension classifier with a wall-clock stability gate and a one-second cooldown.

All three behaviors publish on a shared `/par/intents` topic. The arbiter resolves the winner by `(effective_priority, age, confidence)` per tick and routes the winner through `SafetyLayer.clamp` before publishing on `/cmd_vel`.

## Architecture

Three layers, separated by message contracts rather than node boundaries:

1. **Perception** — `qr_detector`, `perception_fusion`, `gesture_detector`.
2. **Behavior** — `qr_command_interpreter`, `nd_planner`, `recovery_controller`, `anomaly_detector`, `gesture_interpreter`. Each emits velocity intents on `/par/intents`.
3. **Arbitration** — `par_arbiter` resolves a single winner per tick by `(effective_priority, age, confidence)`, then routes through `SafetyLayer.clamp` before publishing on `/cmd_vel`.

The safety layer enforces seven kill paths beneath every command:

| Path | Source | Purpose |
|---|---|---|
| H1 | ToF (0.12 m, directional) | Close-range proximity halt |
| H2 | LIDAR forward halo (±30°, hard 0.20 m, soft 0.20-0.40 m) | Forward halo |
| H2r | LIDAR rear halo (hard 0.10 m, soft 0.10-0.25 m, when reversing) | Asymmetric rear halo, breaks sandwich-geometry deadlock |
| H3 | Arbiter tick watchdog (0.25 s) | Liveness |
| H4 | Stale command drop (0.5 s) | Freshness |
| H5 | Acceleration limiter (0.5 m/s² linear, 3.0 rad/s² angular) | Motion smoothing |
| H6 | Speed cap (`v_max`, `w_max`) | Per-tier ceiling |

The physical e-stop button is the operator-actuated kill above all software paths.

## Packages

| Package | Purpose |
|---|---|
| `par_arbiter` | Priority-with-freshness arbiter and `SafetyLayer` |
| `par_qr_nav` | QR detector and command interpreter |
| `par_reactive_nav` | `perception_fusion`, `nd_planner`, `recovery_controller` |
| `par_gesture` | MediaPipe Hands detector and gesture interpreter |
| `par_anomaly` | Tilt, collision-impact, wheel-impact, wheel-stall detectors with 4-phase `TILT_REVERSE` finite state machine |
| `par_core` | Shared `SafetyLayer`, `BehaviorFSM` base class, telemetry writers |
| `par_msgs` | `CommandIntent`, `DetectionEvent` message types |
| `par_supervisor` | Mode latching and runtime supervision |
| `par_eval` | Offline evaluation tools |
| `par_bringup` | Launch files |

The host-runnable contract test suite contains 325 tests across nine behavior packages.

## Hardware

| Component | Spec |
|---|---|
| Compute | Raspberry Pi 5 (ARM64 quad-core Cortex-A76 @ 2.4 GHz, 8 GB RAM) |
| LIDAR | RPLIDAR S2, 360° at approximately 25 cm above the floor |
| Camera | OAK-D Pro stereo, fixed-mounted approximately 20 cm above the floor |
| IMU | BNO055, nine-axis orientation + linear acceleration + angular rate |
| Range | 4 × VL53L0X time-of-flight sensors at the chassis corners |
| Driver | Husarion `rosbot_bringup` snap on Ubuntu 24.04, ROS 2 Jazzy |

## Documentation

- **[DEPLOYMENT.md](DEPLOYMENT.md)** — first-time setup on a fresh ROSbot 3 PRO, the development container workflow, and how to run the scenes.
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** — common symptoms, root causes, and fixes from real hardware runs.

## License

MIT.
