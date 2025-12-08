# Armatron Notes

## Hardware

- Robot: SO-ARM101
- Motors: Feetech

## Prerequisites

- SO-ARM101 Robot (leader + follower arms)
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

**The arms MUST be plugged in the following order:**

1. **Plug in LEADER first** → assigned `/dev/ttyACM0`
2. **Plug in FOLLOWER second** → assigned `/dev/ttyACM1`

If plugged in the wrong order, unplug both and reconnect in the correct order.

## Calibration

First, activate the lerobot conda environment:

```bash
conda activate lerobot
```

### Leader Arm (on /dev/ttyACM0)

```bash
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=armatron_leader
```

### Follower Arm (on /dev/ttyACM1)

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=armatron
```

## Teleoperation

```bash
lerobot-teleoperate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=armatron_leader \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=armatron
```

## ROS2 Teleoperation with Isaac Sim

The `lerobot-ros-teleoperate` command publishes joint states to ROS2 for real-time visualization in Isaac Sim. Supports two modes:

- **Full mode**: Leader + Follower - follower positions published to ROS2
- **Sim-only mode**: Leader only - leader positions published directly to ROS2

**Requires ROS2 Humble.** For teleoperation without ROS2, use `lerobot-teleoperate` instead.

### Full Mode (Leader + Follower)

Use when you have both physical arms connected:

```bash
source /opt/ros/humble/setup.bash
conda activate lerobot

lerobot-ros-teleoperate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=armatron_leader \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=armatron
```

### Sim-Only Mode (Leader Only)

Use when you want to drive Isaac Sim directly without controlling the physical follower:

```bash
source /opt/ros/humble/setup.bash
conda activate lerobot

lerobot-ros-teleoperate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=armatron_leader
```

Note: Leader positions are published directly to ROS2. The follower arm can remain plugged in but won't be controlled.

### Verifying ROS2 Topics

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /joint_states --once
```

### Using with Isaac Sim

1. Start ROS2 teleoperation (see above)
2. In another terminal, run the topic relay:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run topic_tools relay /joint_states /isaac_joint_command
   ```
3. Open Isaac Sim and load the USD file
4. Press Play - the simulated arm will mirror the physical arm

### Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ros_domain_id` | env var | ROS_DOMAIN_ID (uses `$ROS_DOMAIN_ID` if not specified) |
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

### Gripper Calibration for Isaac Sim

The gripper (Jaw joint) requires special handling because the physical gripper range doesn't align with Isaac Sim's default joint limits.

**The Problem:**
- Physical gripper outputs ~0.02 rad (closed) to ~1.72 rad (open)
- Isaac Sim's Jaw joint needed range down to -11° to fully close

**The Solution:**
1. Set Isaac Sim Jaw joint limits to: **lower: -11, upper: 100**
2. Apply a -0.21 radian offset in code to shift the range down

This maps:
- Physical closed (0.02 rad) → Isaac Sim -0.19 rad (-11°)
- Physical open (1.72 rad) → Isaac Sim 1.51 rad (~87°)

To adjust the gripper offset, edit `src/lerobot/scripts/lerobot_ros_teleoperate.py`:
```python
gripper_offset = -0.21  # Adjust this value if gripper doesn't fully close/open
```

## Why We Built lerobot-ros-teleoperate

### Credits

The Isaac Sim integration concepts and USD model came from [LycheeAI](https://github.com/LycheeAI). His work on the SO-ARM101 ROS2 bridge provided the foundation for this integration. However, the original separate-process architecture didn't work reliably with real-time simulation, so we rewrote the ROS layer to integrate directly into LeRobot's teleoperation loop.

### The Problem: Serial Port Conflicts

Our original architecture for Isaac Sim visualization used a separate `joint_state_reader.py` script that ran independently from teleoperation. The idea was simple: one process handles teleoperation, another reads joint positions and publishes them to ROS2. In theory, this separation of concerns seemed clean.

In practice, it was a disaster.

The Feetech STS3215 motors communicate over USB serial ports using half-duplex communication. When `lerobot-teleoperate` is running, it's constantly reading positions from both arms and writing commands to the follower at 60Hz. The moment we launched `joint_state_reader.py` to read joint positions for Isaac Sim, we introduced a second process trying to access the same serial port (`/dev/ttyACM0`).

The result was immediate and catastrophic:

```
[TxRxResult] There is no status packet!
device reports readiness to read but returned no data (device disconnected or multiple access on port?)
```

The serial port can only handle one reader at a time. Two processes fighting over the same port corrupts the communication protocol, causing packet loss, garbled data, and eventual crashes.

### Failed Approaches

We tried several workarounds:

1. **Mutex locks between processes** - Too slow, still caused timing issues
2. **Separate USB ports** - Would require hardware changes and the follower only has one port
3. **Reduced read frequency** - Still caused intermittent conflicts
4. **Queue-based IPC** - Added latency and complexity without solving the root cause

### The Solution: Integrate ROS2 Publishing into Teleoperation

The realization was that we didn't need two processes reading the same port. The teleoperation loop *already reads the joint positions every cycle* - we just weren't doing anything with that data beyond sending it to the motors.

`lerobot-ros-teleoperate` solves this by publishing joint states to ROS2 from within the teleoperation loop itself:

```
┌─────────────────────────────────────────────────────────┐
│                  Teleoperation Loop                      │
│                                                          │
│   1. Read leader position (ACM0)                        │
│   2. Read follower position (ACM1)  ──► ROS2 Publish    │
│   3. Write to follower (ACM1)                           │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

No port conflicts. No additional processes. The joint states get published at the same 60Hz rate as the control loop, providing smooth real-time visualization in Isaac Sim.

### Key Technical Details

- **Single port access**: Only one process ever touches each serial port
- **Zero additional latency**: Publishing happens inline with the existing read
- **Degrees to radians conversion**: LeRobot uses degrees internally, Isaac Sim expects radians
- **Joint name mapping**: Motor names like `shoulder_pan` map to Isaac Sim joints like `Rotation`

### Lessons Learned

Sometimes the "clean" architecture with separated concerns isn't the right choice. When you're dealing with hardware constraints like serial port access, integrating functionality into existing processes can be simpler and more reliable than orchestrating multiple processes.

## Serial Communication Fix

### Problem

When running teleoperation with Feetech motors, users may encounter intermittent errors:
- `[TxRxResult] There is no status packet!`
- `[TxRxResult] Incorrect status packet!`

This occurs because the original teleop loop order causes a read-after-write timing conflict on the follower's serial port.

**Original problematic order:**
1. Read follower ← fails because it's right after previous iteration's write
2. Read leader
3. Write follower

### Solution

The fix involves three changes:

1. **Reorder teleop loop** (`src/lerobot/scripts/lerobot_teleoperate.py`): Read leader first, then follower, then write. This gives maximum time between write and next read.

2. **Add retries to sync_read** (`src/lerobot/robots/so101_follower/so101_follower.py` and `src/lerobot/teleoperators/so101_leader/so101_leader.py`): Add `num_retry=10` to sync_read calls.

3. **Clear serial buffer before reads** (`src/lerobot/motors/motors_bus.py`): Clear input buffer before each sync_read attempt.

**Fixed order:**
1. Read leader (ACM0)
2. Read follower (ACM1) ← more time since last write
3. Write follower (ACM1)

See: https://github.com/huggingface/lerobot/issues/1252

## Notes

- Position the arm in the middle of its range of motion before calibration
- Ensure external power is connected (USB alone is not sufficient)
- If you get "Incorrect status packet" errors, try running the command again or restart computer
- ROS_DOMAIN_ID must match between all nodes (default: 42)
