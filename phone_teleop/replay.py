#!/usr/bin/env python
"""Replay a phone/WebXR-recorded episode on the SO-101 follower (open-loop IK).

Adapted from lerobot's examples/phone_to_so100/replay.py. Reads the recorded EE
actions and streams them back through IK. No phone/headset needed for replay.

    uv run python phone_teleop/replay.py

Set HF_REPO_ID / EPISODE_IDX to match what record.py wrote.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lerobot.datasets import LeRobotDataset
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import InverseKinematicsEEToJoints
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.constants import ACTION
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say

from so101_config import FOLLOWER_ID, FOLLOWER_PORT, URDF_PATH, URDF_TARGET_FRAME

HF_REPO_ID = "dan/so101_phone_test"
EPISODE_IDX = 0


def main():
    robot_config = SO101FollowerConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID, use_degrees=True)
    robot = SO101Follower(robot_config)

    kinematics_solver = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name=URDF_TARGET_FRAME,
        joint_names=list(robot.bus.motors.keys()),
    )

    robot_ee_to_joints_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=False,  # open loop for replay
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    dataset = LeRobotDataset(HF_REPO_ID, episodes=[EPISODE_IDX])
    actions = dataset.select_columns(ACTION)

    robot.connect()
    try:
        if not robot.is_connected:
            raise ValueError("Robot is not connected!")
        log_say(f"Replaying episode {EPISODE_IDX}")
        for idx in range(dataset.num_frames):
            t0 = time.perf_counter()
            ee_action = {
                name: float(actions[idx][ACTION][i])
                for i, name in enumerate(dataset.features[ACTION]["names"])
            }
            robot_obs = robot.get_observation()
            joint_action = robot_ee_to_joints_processor((ee_action, robot_obs))
            robot.send_action(joint_action)
            precise_sleep(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
