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

## Calibration

### Leader Arm

```bash
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=armatron_leader
```

### Follower Arm

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=armatron
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
