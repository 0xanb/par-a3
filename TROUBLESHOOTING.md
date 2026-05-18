# TROUBLESHOOTING — symptoms and fixes

Common issues observed across hardware testing on the ROSbot 3 PRO, organized by where the symptom shows up first.

## Telemetry and discovery

### Symptom: `./scripts/telemetry.sh up` fails with "discovery failed; cannot bring tunnel up"

The robot's IP address no longer matches the script's expected default. Discover the current address, then override:

```bash
./scripts/find_robot.sh # scans the subnet for SSH responders
ROSBOT_IP=192.168.<x>.<y>./scripts/telemetry.sh up # override
```

Robot IP can change between cold boots if the DHCP lease expires. Run `find_robot.sh` first whenever discovery fails.

### Symptom: `ros2 topic echo --once /par/active_mode` returns `IDLE` even after `scene.sh a`

The default `--once` flag does not request the `transient_local` QoS profile that the supervisor uses to latch the mode. Echo with explicit QoS:

```bash
ros2 topic echo --once /par/active_mode \
 --qos-durability transient_local --qos-reliability reliable
```

This is a ROS 2 client behavior, not a code bug. Behavior nodes that subscribe with the correct QoS see the actual latched value.

## Camera

### Symptom: `/oak/rgb/image_raw` publishes at about 9 Hz but every frame is identical; `qr_detector` logs "camera frozen"

Cold-boot pathology of the `husarion-depthai` snap. Restart the snap and wait about 20 seconds for the OAK driver to re-initialize:

```bash
ssh rosbot@<robot-ip> 'sudo snap restart husarion-depthai && sleep 20'
```

This is repeatable on cold boots and should be the first item in any pre-demo operator checklist.

### Symptom: OAK depth pipeline never publishes on `/oak/stereo/image_raw`

The snap defaults to `i_pipeline_type: RGB` (color only). Flip to RGBD:

```bash
ssh rosbot@<robot-ip> 'sudo sed -i "s|i_pipeline_type: RGB|i_pipeline_type: RGBD|" /var/snap/husarion-depthai/common/camera-params-default.yaml'
ssh rosbot@<robot-ip> 'sudo snap restart husarion-depthai'
```

Mode A enables depth fusion by default; Mode D leaves it off to save CPU.

## Runtime

### Symptom: `par-a3-runtime` service restarts unexpectedly after killing an individual `par_*` node

The systemd unit watches the package group; killing one node with `pkill -f par_` triggers a service-level restart that cycles the entire baseline (about 30 seconds later). Active sessions terminate.

**Rule:** never `pkill` individual `par_*` nodes during a demo. Use `scene.sh` transitions only. If a hot-swap is unavoidable, stop the service first:

```bash
ssh rosbot@<robot-ip> 'sudo systemctl stop par-a3-runtime'
```

then run nodes by hand, then restart the service when done.

### Symptom: Mode D starts but Foxglove disconnects

By design. The Mode D CPU policy stops the `foxglove_bridge` snap to free a core for MediaPipe Hands inference. The operator's view during a gesture demonstration is the robot in front of them, not Foxglove.

Returning to Mode A or stopping the scene restores the bridge automatically:

```bash
./scripts/scene.sh stop
```

### Symptom: Robot is in `SELF_VALIDATE` indefinitely; the supervisor never reaches `IDLE`

`/joint_states` is silent. This usually means the motor driver is in under-voltage lockout because the battery is low. Check the rear-panel L1 LED:

| L1 LED state | Meaning |
|---|---|
| Solid | Battery healthy, motors armed |
| Blinking | Battery low, motor driver in under-voltage lockout |

Charge to solid-L1 before retrying the scene.

## Reactive navigation

### Symptom: Robot stops in front of an obstacle and refuses to back up

The forward LIDAR halo (H2) and the rear LIDAR halo (H2r) are sandwiching the chassis between a forward obstacle and a rear wall. The asymmetric split (forward 0.20 m, rear 0.10 m) is the deployed fix; check that the deployed parameters match the table in `README.md`:

```bash
ros2 param get /par_arbiter lidar_rear_stop_m # expect 0.10
ros2 param get /par_arbiter lidar_rear_slow_m # expect 0.25
```

If the rear halo is too tight, soften by passing different values to the launch file. The H2r consultation is gated on the resolved winner having `linear.x < 0`, so ordinary forward motion is unaffected.

### Symptom: Recovery FSM fires but the chassis does not move

The recovery reverse velocity is being clamped by the H1 ToF halo (0.12 m). Confirm:

```bash
ros2 topic echo /par/events --once # look for clamp_reason="H1_TOF_REAR"
```

If H1 is firing on a phantom rear obstacle (sensor noise on smooth floor), increase `tof_min_m` slightly or check the rear ToF sensors physically.

### Symptom: `dead_end_blocked_frac` triggers too eagerly in narrow corridors

The deployed value is 0.55, equivalent to five-of-eight forward-cone bins blocked. Tighten to 0.70 (six-of-eight) for a more permissive planner:

```bash
ros2 param set /nd_planner dead_end_blocked_frac 0.70
```

The trade-off is that wider corridors will still escape dead-end declaration; narrower corridors will see more wedge-watchdog firings instead.

## Gesture (Mode D)

### Symptom: Hand is detected but no verb fires

MediaPipe Hands is returning landmarks but the rule-based classifier is rejecting them. The most common cause is operator distance outside the reliable envelope. The classifier is tuned for a seated operator at 0.6 to 1.0 m from the camera with a hand-size that matches the trial operator's geometry.

Check the candidate label stream:

```bash
ros2 topic echo /par/detections | grep gesture
```

If labels are flickering, the wall-clock stability gate (0.4 s of contiguous same-label detections) is suppressing them. Hold the pose steadier.

### Symptom: Gun-pose verbs (TURN_LEFT, TURN_RIGHT) mis-classify as PEACE or THUMBS_UP

Known limitation of single-operator threshold tuning. The geometric template overlap between gun-pose and peace varies with finger-length ratios. The deployed system avoids gun-pose verbs in the live demonstration; the five robust verbs (CLOSED_FIST, OK, PEACE, THUMBS_UP, THUMBS_DOWN) cover the operational vocabulary.

A hand-size-aware threshold normalization would resolve this; see the project's future-work notes.

## Build

### Symptom: `colcon build` fails on `par_arbiter` with "Could not find a package configuration file provided by par_msgs"

The `par_msgs` package must build before any package that imports `CommandIntent` or `DetectionEvent`. Build in dependency order:

```bash
colcon build --packages-select par_msgs
source install/setup.bash
colcon build
```

Or use the dev container's `./scripts/dev.sh test` which handles the order automatically.

### Symptom: `colcon test` reports 324 of 325 tests passing

One package's tests need a fresh `install/`. Clean and re-test:

```bash
rm -rf workspace/build workspace/install workspace/log
colcon build
colcon test
```

If still 324, identify the failing package and re-run that suite specifically.
