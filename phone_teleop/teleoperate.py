#!/usr/bin/env python
"""Phone / WebXR teleop of the SO-101 follower (single arm, end-effector control).

Adapted from lerobot's examples/phone_to_so100/teleoperate.py for this repo:
- SO101Follower on the port/id from so101_config
- PhoneOS.ANDROID (WebXR) so it works in the Quest 2 browser (no APK sideload)
- URDF from so101_config (run ../setup.sh first)

    uv run python phone_teleop/teleoperate.py

Then open the printed URL in the Quest browser, tap Start, and press-and-hold
"Move" to drive the arm. The first press latches the reference pose (the clutch).
See phone_teleop/README.md.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # phone_teleop/ on path
import _hebi_stub  # noqa: F401,E402  -- MUST precede any lerobot.teleoperators.phone import

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.phone import Phone, PhoneConfig
from lerobot.teleoperators.phone.config_phone import PhoneOS
from lerobot.teleoperators.phone.phone_processor import MapPhoneActionToRobotAction
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from so101_config import FOLLOWER_ID, FOLLOWER_PORT, URDF_PATH, URDF_TARGET_FRAME

FPS = 30


def main():
    robot_config = SO101FollowerConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID, use_degrees=True)
    teleop_config = PhoneConfig(phone_os=PhoneOS.ANDROID)  # WebXR (Quest browser)

    robot = SO101Follower(robot_config)
    teleop_device = Phone(teleop_config)

    kinematics_solver = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name=URDF_TARGET_FRAME,
        joint_names=list(robot.bus.motors.keys()),
    )

    phone_to_robot_joints_processor = RobotProcessorPipeline[
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
                max_ee_step_m=0.10,
            ),
            GripperVelocityToJoint(speed_factor=20.0),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=True,  # closed loop
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot.connect()
    teleop_device.connect()
    init_rerun(session_name="so101_phone_teleop")

    if not robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    print("Teleop loop started. Open the printed URL in the Quest browser, hold Move to drive.")
    try:
        while True:
            t0 = time.perf_counter()
            robot_obs = robot.get_observation()
            phone_obs = teleop_device.get_action()
            if not phone_obs:  # no pose yet — don't crash the pipeline
                precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
                continue
            joint_action = phone_to_robot_joints_processor((phone_obs, robot_obs))
            robot.send_action(joint_action)
            log_rerun_data(observation=phone_obs, action=joint_action)
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
    finally:
        robot.disconnect()
        teleop_device.disconnect()


if __name__ == "__main__":
    main()
