# SO-ARM101 Real2Sim Teleoperation with Isaac Sim

Complete guide for setting up SO-ARM101 robot teleoperation with Isaac Sim via ROS2.

## Prerequisites

- SO-ARM101 Dual-Arm Robot
- LeRobot library installed (`pip install -e '.[feetech]'`)
- Isaac Sim 4.5 or 5.0
- ROS2 Humble
- Ubuntu 22.04

## Step 1: Physical Robot Setup

### USB Permissions

Grant access to the robot's serial ports:

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

**Permanent fix** - Add yourself to the dialout group:

```bash
sudo usermod -aG dialout $USER
```
Then log out and back in.

### Port Assignment

Ports can swap between reboots. To ensure consistent assignment:

1. **Always plug in leader first** → gets `/dev/ttyACM0`
2. **Plug in follower second** → gets `/dev/ttyACM1`

Or use `lerobot-find-port` to identify which arm is on which port.

### Troubleshooting: "Incorrect status packet" Errors

If you get intermittent serial communication errors:

1. **Restart your computer** - USB serial ports can get into a bad state
2. **Check servo chain cables** - Loose connections cause intermittent failures
3. **Check power supply** - Motors need adequate current
4. **Try different USB cables** - Some cables have poor data quality
5. **Avoid USB hubs** - Plug directly into computer

### LeRobot Calibration

Calibrate each arm before teleoperation:

```bash
# Activate lerobot environment
conda activate lerobot

# Calibrate leader arm
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=leader

# Calibrate follower arm
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=follower
```

**Important**: Position the arm in the middle of its range of motion before pressing ENTER during calibration.

### LeRobot Teleoperation Test

Verify hardware works before Isaac Sim integration:

```bash
conda activate lerobot

lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader
```

## Step 2: ROS2 Workspace Setup

### Install Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-humble-topic-tools
```

### Clone and Build

```bash
cd ~/source/lerobot
git clone https://github.com/MuammerBay/so-arm101-ros2-bridge.git
cd so-arm101-ros2-bridge

# Build ROS2 workspace
colcon build --packages-select jointstatereader
source install/setup.bash
```

### Launch Joint State Reader

**Important**: Run this from the lerobot conda environment to avoid Python version conflicts.

```bash
conda activate lerobot
python3 src/jointstatereader/jointstatereader/joint_state_reader.py
```

You should see:
```
[INFO] Connected to SO100 robot on /dev/ttyACM0
[INFO] SO100 Hardware Driver started - publishing to /joint_states at 20Hz
```

### Setup Topic Relay for Isaac Sim

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run topic_tools relay /joint_states /isaac_joint_command
```

### Verify ROS2 Topics

```bash
# List all topics
ros2 topic list

# Check joint states are publishing
ros2 topic echo /joint_states --once

# Check relay is working
ros2 topic echo /isaac_joint_command --once
```

## Step 3: Isaac Sim Configuration

### Launch Isaac Sim

**Isaac Sim 5.0:**
```bash
cd ~/isaacsim/_build/linux-x86_64/release && ./isaac-sim.sh
```

**Isaac Sim 4.5:**
```bash
~/IsaacSim_4-5/isaac-sim.sh
```

### Load USD Model

1. In Isaac Sim, go to **File → Open**
2. Navigate to: `~/source/lerobot/so-arm101-ros2-bridge/IsaacSim_USD/SO-ARM101-USD.usd`
3. Press **Play** to start physics simulation and ROS2 integration

## Joint Name Mapping

The joint_state_reader publishes with these joint names:

| Servo ID | Joint Name   | Description      |
|----------|--------------|------------------|
| 1        | Rotation     | Base rotation    |
| 2        | Pitch        | Shoulder pitch   |
| 3        | Elbow        | Elbow            |
| 4        | Wrist_Pitch  | Wrist pitch      |
| 5        | Wrist_Roll   | Wrist roll       |
| 6        | Jaw          | Gripper          |

Ensure your Isaac Sim action graph matches these joint names.

## Common Issues

### Python Version Mismatch

**Error**: `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`

**Solution**: Run from the lerobot conda environment, not base:
```bash
conda activate lerobot
python3 src/jointstatereader/jointstatereader/joint_state_reader.py
```

### topic_tools Not Found

**Error**: `Package 'topic_tools' not found`

**Solution**: Install the package:
```bash
sudo apt-get install ros-humble-topic-tools
```

### Motors Not Responding

**Error**: `Missing motor IDs` or `Incorrect status packet`

**Solutions**:
1. Restart computer
2. Check all servo cable connections
3. Verify power supply is connected
4. Run `lerobot-find-port` to confirm correct port

### Calibration Offset Errors

**Error**: `Magnitude exceeds max` during calibration

**Solution**: Position arm closer to center of range before calibration.

## Architecture Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│  SO-ARM101      │     │  joint_state_    │     │  Isaac Sim  │
│  Hardware       │────▶│  reader.py       │────▶│  USD Stage  │
│  /dev/ttyACM0   │     │  /joint_states   │     │             │
└─────────────────┘     └──────────────────┘     └─────────────┘
                               │
                               ▼
                        ┌──────────────────┐
                        │  topic_tools     │
                        │  relay           │
                        │  /isaac_joint_   │
                        │  command         │
                        └──────────────────┘
```

## Quick Start Checklist

1. [ ] Leader arm plugged in first (ACM0)
2. [ ] Follower arm plugged in second (ACM1)
3. [ ] Both arms calibrated with lerobot-calibrate
4. [ ] Teleoperation test passes with lerobot-teleoperate
5. [ ] ROS2 Humble sourced
6. [ ] topic_tools installed
7. [ ] joint_state_reader running (in lerobot conda env)
8. [ ] topic relay running
9. [ ] Isaac Sim loaded with USD file
10. [ ] Isaac Sim Play button pressed
