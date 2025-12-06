# Armatron Notes

## Hardware

- Robot: SO-ARM101
- Motors: Feetech

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

## Notes

- Position the arm in the middle of its range of motion before calibration
- Ensure external power is connected (USB alone is not sufficient)
