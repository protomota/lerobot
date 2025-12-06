# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Dual teleoperation with ROS2 joint state publishing for Isaac Sim integration.

This script teleoperates the robot while publishing joint states to ROS2
for real-time visualization in Isaac Sim.

Requires ROS2 Humble. For teleoperation without ROS2, use lerobot-teleoperate instead.

Example:

```shell
source /opt/ros/humble/setup.bash
conda activate lerobot

lerobot-dual-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=armatron \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=armatron_leader
```

"""

import logging
import math
import os
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import rerun as rr

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so100_follower,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    gamepad,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
)
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

# ROS2 imports - required for this script
try:
    import rclpy
    from sensor_msgs.msg import JointState
except ImportError as e:
    raise ImportError(
        "ROS2 is required for lerobot-dual-teleoperate. "
        "Please install ROS2 Humble and run: source /opt/ros/humble/setup.bash\n"
        "For teleoperation without ROS2, use lerobot-teleoperate instead."
    ) from e


# Joint name mapping from lerobot motor names to Isaac Sim joint names
LEROBOT_TO_ISAAC_JOINT_MAP = {
    "shoulder_pan": "Rotation",
    "shoulder_lift": "Pitch",
    "elbow_flex": "Elbow",
    "wrist_flex": "Wrist_Pitch",
    "wrist_roll": "Wrist_Roll",
    "gripper": "Jaw",
}

ISAAC_JOINT_ORDER = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]


class JointStatePublisher:
    """Publishes joint states to ROS2 /joint_states topic for Isaac Sim integration."""

    def __init__(self, ros_domain_id: int = 42):
        """Initialize ROS2 node and publisher.

        Args:
            ros_domain_id: ROS_DOMAIN_ID for Isaac Sim communication (default: 42)
        """
        # Set ROS_DOMAIN_ID before initializing rclpy
        os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)

        rclpy.init()
        self.node = rclpy.create_node("dual_teleop_publisher")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.joint_names = ISAAC_JOINT_ORDER

        logging.info(f"ROS2 JointStatePublisher initialized on domain {ros_domain_id}")
        logging.info(f"Publishing to /joint_states with joints: {self.joint_names}")

    def publish_joint_states(self, follower_obs: dict[str, float]) -> None:
        """Publish follower joint states to ROS2.

        Args:
            follower_obs: Dictionary of follower observations with keys like "shoulder_pan.pos"
        """
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self._convert_to_isaac_positions(follower_obs)
        self.publisher.publish(msg)

    def _convert_to_isaac_positions(self, obs_dict: dict[str, float]) -> list[float]:
        """Convert lerobot observation dict to Isaac Sim joint positions in radians.

        Args:
            obs_dict: Dictionary with keys like "shoulder_pan.pos" and normalized values

        Returns:
            List of joint positions in radians, ordered for Isaac Sim
        """
        positions = []
        for isaac_name in ISAAC_JOINT_ORDER:
            # Find corresponding lerobot name
            lerobot_name = None
            for lr_name, is_name in LEROBOT_TO_ISAAC_JOINT_MAP.items():
                if is_name == isaac_name:
                    lerobot_name = lr_name
                    break

            if lerobot_name is None:
                positions.append(0.0)
                continue

            key = f"{lerobot_name}.pos"
            if key in obs_dict:
                # lerobot uses normalized values, convert to radians
                # Normalized range is typically -1 to 1, maps to -pi to pi
                value = obs_dict[key]
                radians = value * math.pi
                positions.append(radians)
            else:
                positions.append(0.0)

        return positions

    def shutdown(self) -> None:
        """Clean shutdown of ROS2 node."""
        logging.info("Shutting down ROS2 JointStatePublisher")
        self.node.destroy_node()
        rclpy.shutdown()


@dataclass
class DualTeleoperateConfig:
    """Configuration for dual teleoperation with ROS2 joint state publishing."""

    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 60
    teleop_time_s: float | None = None
    display_data: bool = False
    ros_domain_id: int = 42


def dual_teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    display_data: bool = False,
    duration: float | None = None,
    ros2_publisher: JointStatePublisher | None = None,
):
    """
    Teleoperation loop with optional ROS2 joint state publishing for Isaac Sim.

    CRITICAL: Maintains the read leader -> read follower -> write follower order
    to avoid serial communication conflicts on dual-arm setups.

    Args:
        teleop: The teleoperator device instance providing control actions.
        robot: The robot instance being controlled.
        fps: The target frequency for the control loop in frames per second.
        display_data: If True, fetches robot observations and displays them in the console and Rerun.
        duration: The maximum duration of the teleoperation loop in seconds. If None, runs indefinitely.
        teleop_action_processor: Pipeline to process raw actions from the teleoperator.
        robot_action_processor: Pipeline to process actions before they are sent to the robot.
        robot_observation_processor: Pipeline to process raw observations from the robot.
        ros2_publisher: Optional ROS2 publisher for joint states.
    """
    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()

    while True:
        loop_start = time.perf_counter()

        # Step 1: Get teleop action FIRST (read from leader on ACM1)
        raw_action = teleop.get_action()

        # Step 2: Get robot observation AFTER leader read, BEFORE write
        # This ordering gives maximum time between write and next read cycle
        # to avoid dual-arm serial conflicts (see github.com/huggingface/lerobot/issues/1252)
        obs = robot.get_observation()

        # Step 3: Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Step 4: Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # Step 5: Send processed action to robot (write to follower on ACM0)
        _ = robot.send_action(robot_action_to_send)

        # Step 6: Publish to ROS2 if enabled (after write, before next loop)
        if ros2_publisher is not None:
            ros2_publisher.publish_joint_states(obs)

        if display_data:
            # Process robot observation through pipeline
            obs_transition = robot_observation_processor(obs)

            log_rerun_data(
                observation=obs_transition,
                action=teleop_action,
            )

            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'NORM':>7}")
            # Display the final robot action that was sent
            for motor, value in robot_action_to_send.items():
                print(f"{motor:<{display_len}} | {value:>7.2f}")
            move_cursor_up(len(robot_action_to_send) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(1 / fps - dt_s)
        loop_s = time.perf_counter() - loop_start
        print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
        move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def dual_teleoperate(cfg: DualTeleoperateConfig):
    """Main entry point for dual teleoperation with ROS2 joint state publishing."""
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.display_data:
        init_rerun(session_name="dual_teleoperation")

    # Initialize ROS2 publisher
    ros2_publisher = JointStatePublisher(ros_domain_id=cfg.ros_domain_id)

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    teleop.connect()
    robot.connect()

    try:
        dual_teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            ros2_publisher=ros2_publisher,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        robot.disconnect()
        ros2_publisher.shutdown()


def main():
    register_third_party_devices()
    dual_teleoperate()


if __name__ == "__main__":
    main()
