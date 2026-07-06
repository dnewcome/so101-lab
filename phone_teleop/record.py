#!/usr/bin/env python
"""Record a LeRobotDataset by phone/WebXR teleop of the SO-101 follower.

Adapted from lerobot's examples/phone_to_so100/record.py for this repo. Saves
absolute end-effector observations + actions, so the dataset trains/replays
through lerobot's EE pipeline (see replay.py, then `lerobot-train`).

    uv run python phone_teleop/record.py

Edit the CONFIG block below (task, episodes, repo id). By default it stays LOCAL
(no Hub push). Camera streams MJPG on its own USB port (see docs/SO101_BRINGUP.md
-- raw YUYV starves the servo bus).

Wayland note: episode next/redo/stop keys use pynput, which Wayland blocks, so
episodes advance on the EPISODE_TIME_SEC timer (same as lerobot-record here).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import init_keyboard_listener
from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotProcessorPipeline,
    observation_to_transition,
    robot_action_observation_to_transition,
    transition_to_observation,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    ForwardKinematicsJointsToEE,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.phone import Phone, PhoneConfig
from lerobot.teleoperators.phone.config_phone import PhoneOS
from lerobot.teleoperators.phone.phone_processor import MapPhoneActionToRobotAction
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

from so101_config import FOLLOWER_ID, FOLLOWER_PORT, URDF_PATH, URDF_TARGET_FRAME

# ----------------------------- CONFIG (edit me) ----------------------------- #
NUM_EPISODES = 10
FPS = 30
EPISODE_TIME_SEC = 30
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "pick up the orange bottle cap and drop it in the light blue bin"
HF_REPO_ID = "dan/so101_phone_test"
PUSH_TO_HUB = False  # True needs `huggingface-cli login`
CAMERA_INDEX = 0
# ---------------------------------------------------------------------------- #


def main():
    camera_config = {
        "front": OpenCVCameraConfig(index_or_path=CAMERA_INDEX, width=640, height=480, fps=FPS, fourcc="MJPG")
    }
    robot_config = SO101FollowerConfig(
        port=FOLLOWER_PORT, id=FOLLOWER_ID, cameras=camera_config, use_degrees=True
    )
    teleop_config = PhoneConfig(phone_os=PhoneOS.ANDROID)  # WebXR (Quest browser)

    robot = SO101Follower(robot_config)
    phone = Phone(teleop_config)

    kinematics_solver = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name=URDF_TARGET_FRAME,
        joint_names=list(robot.bus.motors.keys()),
    )

    # phone pose -> EE-pose action (+ gripper velocity->joint)
    phone_to_robot_ee_pose_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=[
            MapPhoneActionToRobotAction(platform=teleop_config.phone_os),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                motor_names=list(robot.bus.motors.keys()),
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.20,
            ),
            GripperVelocityToJoint(speed_factor=20.0),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # EE-pose action -> joints (IK)
    robot_ee_to_joints_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # joint observation -> EE observation (FK), so state.ee.* is logged for training
    robot_joints_to_ee_pose = RobotProcessorPipeline[RobotObservation, RobotObservation](
        steps=[ForwardKinematicsJointsToEE(kinematics=kinematics_solver, motor_names=list(robot.bus.motors.keys()))],
        to_transition=observation_to_transition,
        to_output=transition_to_observation,
    )

    dataset = LeRobotDataset.create(
        repo_id=HF_REPO_ID,
        fps=FPS,
        features=combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=phone_to_robot_ee_pose_processor,
                initial_features=create_initial_features(action=phone.action_features),
                use_videos=True,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_joints_to_ee_pose,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=True,
            ),
        ),
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    robot.connect()
    phone.connect()
    listener, events = init_keyboard_listener()
    init_rerun(session_name="so101_phone_record")

    try:
        if not robot.is_connected or not phone.is_connected:
            raise ValueError("Robot or teleop is not connected!")

        print("Record loop started. Open the printed URL in the Quest browser, hold Move to drive.")
        episode_idx = 0
        while episode_idx < NUM_EPISODES and not events["stop_recording"]:
            log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")
            record_loop(
                robot=robot, events=events, fps=FPS,
                teleop_action_processor=phone_to_robot_ee_pose_processor,
                robot_action_processor=robot_ee_to_joints_processor,
                robot_observation_processor=robot_joints_to_ee_pose,
                teleop=phone, dataset=dataset,
                control_time_s=EPISODE_TIME_SEC, single_task=TASK_DESCRIPTION, display_data=True,
            )
            if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
                log_say("Reset the environment")
                record_loop(
                    robot=robot, events=events, fps=FPS,
                    teleop_action_processor=phone_to_robot_ee_pose_processor,
                    robot_action_processor=robot_ee_to_joints_processor,
                    robot_observation_processor=robot_joints_to_ee_pose,
                    teleop=phone, control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION, display_data=True,
                )
            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue
            dataset.save_episode()
            episode_idx += 1
    finally:
        log_say("Stop recording")
        robot.disconnect()
        phone.disconnect()
        listener.stop()
        dataset.finalize()
        if PUSH_TO_HUB:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
