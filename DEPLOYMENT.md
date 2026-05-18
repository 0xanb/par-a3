# DEPLOYMENT — first-time setup and run

This guide takes a fresh Husarion ROSbot 3 PRO from out-of-the-box to a running two-mode demonstration.

## Prerequisites

**Developer machine:**

- macOS or Linux with Docker. On macOS the dev container runs through Colima.
- SSH access to the robot (the developer machine and the robot share a network).
- About 10 GB free disk for the dev image plus the workspace.

**Robot:**

- Husarion ROSbot 3 PRO with Ubuntu 24.04 and the Husarion `rosbot_bringup` snap pre-installed.
- A wired or wireless network connection that the developer machine can reach.

## 1. First-time developer machine setup

Clone the repository, then bring up the dev container.

```bash
git clone https://github.com/0xanb/par-a3.git
cd par-a3
./scripts/dev.sh up
```

On macOS this starts Colima with a four-core ARM virtual machine, builds the ROS 2 Jazzy container image, and opens a shell inside. On Linux it uses native Docker.

Verify the container is healthy by running the contract test suite:

```bash
./scripts/dev.sh test
```

The full suite is 325 host-runnable tests across nine behavior packages; all must pass.

## 2. First-time robot provisioning

The robot needs a small one-time configuration step before the deployment workflow can drive it.

### DDS profile

Robot-side ROS 2 nodes must share a Fast DDS profile that the snap-based `rosbot_bringup` driver also uses. Copy the profile into place:

```bash
ssh rosbot@<robot-ip> 'sudo mkdir -p ~/.config/fastdds && sudo cp /var/snap/husarion-depthai/common/dds-config-udp.xml ~/.config/fastdds/par-a3.xml'
```

If either the `husarion-depthai` or `husarion-webui` snap was provisioned with the loopback DDS profile (`dds-config-udp-lo.xml`), switch them to the non-loopback profile so the dev machine can see their topics:

```bash
ssh rosbot@<robot-ip> 'sudo snap set husarion-depthai dds-profile=dds-config-udp.xml'
ssh rosbot@<robot-ip> 'sudo snap set husarion-webui dds-profile=dds-config-udp.xml'
```

### Workspace bootstrap

Push the workspace to the robot and build:

```bash
./scripts/deploy.sh
```

This copies the workspace, runs `colcon build`, and reports back. Check the report-back for green builds across all `par_*` packages.

### Always-on systemd service

The `par-a3-runtime` systemd service brings up the always-on baseline (arbiter, QR pipeline, recorder, session logger) at boot. The unit file is installed by `deploy.sh` on first run; enable and start it:

```bash
ssh rosbot@<robot-ip> 'sudo systemctl enable par-a3-runtime && sudo systemctl start par-a3-runtime'
```

Verify:

```bash
ssh rosbot@<robot-ip> 'systemctl status par-a3-runtime'
```

Expect `active (running)` within about 45 seconds (the supervisor performs a 360° readiness rotation on first start).

## 3. Daily workflow

### Push code changes

After any local edit, push to the robot and rebuild:

```bash
./scripts/deploy.sh # push + colcon build
./scripts/deploy.sh status # confirm what commit landed on the robot
```

The deploy step refuses if the local tree is dirty or the robot tree is ahead of origin; the script is the single source of truth for what the robot is running.

### Bring up the operator telemetry tunnel

```bash
./scripts/telemetry.sh up
```

This opens an SSH tunnel from `localhost:8766` to the robot's `foxglove_bridge`. Connect Foxglove Studio to `ws://localhost:8766`.

### Select a scene

The always-on baseline runs continuously. The operator selects an active behavior with `scene.sh`:

```bash
./scripts/scene.sh a # Mode A: reactive navigation + QR pause/resume
./scripts/scene.sh d # Mode D: hand-gesture command
./scripts/scene.sh stop # Return to standby (arbiter still ticking)
```

Mode A and Mode D are mutually exclusive on this compute budget. Mode D's CPU policy stops the QR detector and the Foxglove bridge to free a core for MediaPipe Hands inference.

### Run a trial

The trial harness logs raw event streams plus a per-trial `metrics.yaml`:

```bash
./scripts/run_trial.sh <algorithm> <sensor_cell> integrated 150
```

Outputs land under `~/par_ws/logs/<session>/<trial_id>/` on the robot. Pull them back with `rsync` for offline analysis with `scripts/build_demo_logs.py` and `scripts/plot_aligned_paths.py`.

## 4. Pre-demo preflight

Before any live demonstration, run the preflight from the developer machine:

```bash
./scripts/preflight.sh
```

The script walks the operator through a seven-stage check: workspace builds green, contract test suite passes, all expected ROS 2 packages are registered, all sensor topics are publishing at the expected rates, safety invariants fire on synthetic inputs, then a human checklist (battery LED solid, e-stop reachable, arena clear).

## 5. Cold-boot recovery

If the robot has been powered off, the OAK-D camera occasionally needs a soft restart of its snap to publish fresh frames. See `TROUBLESHOOTING.md`.
