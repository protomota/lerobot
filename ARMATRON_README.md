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

## Isaac Sim Integration

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

## Troubleshooting

### "Incorrect status packet" or "There is no status packet" Errors

These errors indicate intermittent serial communication failures with the Feetech motors. Try the following in order:

1. **Restart the computer** - This is often the most effective fix. USB serial ports can get into a bad state that persists until reboot.

2. **Check power supply** - Ensure external power is connected. USB alone is not sufficient to power the motors reliably.

3. **Check cable connections** - Loose servo chain cables cause intermittent failures. Firmly reseat all motor connections.

4. **Use direct USB ports** - Avoid USB hubs. Plug directly into the computer.

5. **Try different USB cables** - Some cables have poor data quality.

6. **Reset USB subsystem**:
   ```bash
   sudo modprobe -r cdc_acm && sudo modprobe cdc_acm
   ```

7. **Unplug and replug both arms** - Remember the correct order: follower first (ACM0), then leader (ACM1).

### Known Issue: Dual-Arm Teleoperation Instability

There is a known intermittent communication issue when running dual-arm teleoperation with Feetech motors. The symptoms include:
- Teleoperation failing after a few successful iterations
- Errors like `[TxRxResult] There is no status packet!`
- Communication getting stuck in a bad state

**Workarounds:**
- A computer restart often resolves the issue temporarily
- Individual arm operations (calibration, single-arm testing) work reliably
- If teleoperation fails, try restarting and running the command again

This appears to be a timing/driver issue with the Feetech servo communication when using sync_read/sync_write operations across multiple serial ports simultaneously.

## Notes

- Position the arm in the middle of its range of motion before calibration
- Ensure external power is connected (USB alone is not sufficient)
- ROS_DOMAIN_ID must match between all nodes (default: 42)
