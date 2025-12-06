# Armatron Notes

## Hardware

- Robot: SO-ARM101
- Motors: Feetech

## Prerequisites

- SO-ARM101 Dual-Arm Robot
- LeRobot library installed
- Isaac Sim 4.5 or 5.0 (for simulation)
- ROS2 Humble
- so-arm101-ros2-bridge (included in this repo)

## Setup

### Installation

```bash
pip install -e '.[feetech]'
```

### Permissions

Add user to dialout group for serial port access:

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

Or temporarily grant access:

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

### Find Port

```bash
lerobot-find-port
```

## IMPORTANT: USB Connection Order

**The arms MUST be plugged in the following order for teleoperation to work:**

1. **Plug in FOLLOWER first** → assigned `/dev/ttyACM0`
2. **Plug in LEADER second** → assigned `/dev/ttyACM1`

If plugged in the wrong order, unplug both and reconnect in the correct order.

## Calibration

First, activate the lerobot conda environment:

```bash
conda activate lerobot
```

### Follower Arm (on /dev/ttyACM0)

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=armatron
```

### Leader Arm (on /dev/ttyACM1)

```bash
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=armatron_leader
```

## Teleoperation

```bash
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=armatron \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=armatron_leader
```

## Dual Teleoperation with Isaac Sim

The `lerobot-dual-teleoperate` command combines standard teleoperation with ROS2 joint state publishing, allowing real-time visualization in Isaac Sim while teleoperating.

### Basic Usage (without ROS2)

```bash
lerobot-dual-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=armatron \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=armatron_leader
```

### With ROS2 Publishing for Isaac Sim

```bash
# Source ROS2 first, then activate conda environment
source /opt/ros/humble/setup.bash
conda activate lerobot

lerobot-dual-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=armatron \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=armatron_leader \
    --ros2_publish=true \
    --ros_domain_id=42
```

### Verifying ROS2 Topics

In another terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
ros2 topic echo /joint_states --once
```

### Using with Isaac Sim

1. Start dual teleoperation with `--ros2_publish=true`
2. In another terminal, run the topic relay:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 run topic_tools relay /joint_states /isaac_joint_command
   ```
3. Open Isaac Sim and load the USD file
4. Press Play - the simulated arm will mirror the physical follower arm

### Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ros2_publish` | `false` | Enable ROS2 joint state publishing |
| `--ros_domain_id` | `42` | ROS_DOMAIN_ID for Isaac Sim communication |
| `--fps` | `60` | Control loop frequency (also publishing rate) |

### Joint Name Mapping

The script maps lerobot motor names to Isaac Sim joint names:

| LeRobot Motor | Isaac Sim Joint |
|---------------|-----------------|
| shoulder_pan | Rotation |
| shoulder_lift | Pitch |
| elbow_flex | Elbow |
| wrist_flex | Wrist_Pitch |
| wrist_roll | Wrist_Roll |
| gripper | Jaw |

## Isaac Sim Integration (Legacy Method)

### Step 1: Build ROS2 Workspace

```bash
cd ~/source/lerobot/so-arm101-ros2-bridge
colcon build --packages-select jointstatereader
source install/setup.bash
```

### Step 2: Install topic_tools (if not already installed)

```bash
sudo apt-get install ros-humble-topic-tools
```

### Step 3: Launch Joint State Reader

Run from lerobot conda environment:

```bash
conda activate lerobot
python3 src/jointstatereader/jointstatereader/joint_state_reader.py
```

### Step 4: Start Topic Relay

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 run topic_tools relay /joint_states /isaac_joint_command
```

### Step 5: Verify Topics

```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /isaac_joint_command --once
```

### Step 6: Launch Isaac Sim

**Isaac Sim 5.0:**
```bash
cd ~/isaacsim/_build/linux-x86_64/release && ./isaac-sim.sh
```

**Isaac Sim 4.5:**
```bash
~/IsaacSim_4-5/isaac-sim.sh
```

### Step 7: Open USD File

Open the USD file in Isaac Sim:

```
~/source/lerobot/so-arm101-ros2-bridge/IsaacSim_USD/SO-ARM101-USD.usd
```

Press **Play** to start physics simulation and ROS2 integration.

## Dual-Arm Serial Communication Fix

### Problem

When running dual-arm teleoperation with Feetech motors, users may encounter intermittent errors:
- `[TxRxResult] There is no status packet!`
- `[TxRxResult] Incorrect status packet!`

This occurs because the original teleop loop order causes a read-after-write timing conflict on the follower's serial port (ACM0).

**Original problematic order:**
1. Read follower (ACM0) ← fails because it's right after previous iteration's write
2. Read leader (ACM1)
3. Write follower (ACM0)

### Solution

The fix involves three changes:

1. **Reorder teleop loop** (`src/lerobot/scripts/lerobot_teleoperate.py`): Read leader first, then follower, then write. This gives maximum time between write and next read.

2. **Add retries to sync_read** (`src/lerobot/robots/so101_follower/so101_follower.py` and `src/lerobot/teleoperators/so101_leader/so101_leader.py`): Add `num_retry=10` to sync_read calls.

3. **Clear serial buffer before reads** (`src/lerobot/motors/motors_bus.py`): Clear input buffer before each sync_read attempt.

**Fixed order:**
1. Read leader (ACM1)
2. Read follower (ACM0) ← more time since last write
3. Write follower (ACM0)

See: https://github.com/huggingface/lerobot/issues/1252

## Notes

- Position the arm in the middle of its range of motion before calibration
- Ensure external power is connected (USB alone is not sufficient)
- If you get "Incorrect status packet" errors, try running the command again or restart computer
- ROS_DOMAIN_ID must match between all nodes (default: 42)
