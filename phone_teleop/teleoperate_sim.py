#!/usr/bin/env python
"""Phone / WebXR teleop driving the SO-101 in SIMULATION (MuJoCo).

Same lerobot phone pipeline as teleoperate.py -- only the robot is swapped for a
MuJoCo `SimRobot`, so you can dial in the WebXR loop with NO hardware follower.
You still need the phone/Quest for the pose input; the arm you drive is virtual.

    uv run python phone_teleop/teleoperate_sim.py           # opens a MuJoCo window
    uv run python phone_teleop/teleoperate_sim.py --no-view  # headless

Open the printed URL in the Quest browser, tap Start, hold Move to drive.
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

from sim_robot import SimRobot
from so101_config import URDF_PATH, URDF_TARGET_FRAME

FPS = 30


def main():
    view = "--no-view" not in sys.argv
    robot = SimRobot(view=view)
    teleop_config = PhoneConfig(phone_os=PhoneOS.ANDROID)  # WebXR (Quest browser)
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
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot.connect()
    teleop_device.connect()
    init_rerun(session_name="so101_phone_teleop_sim")

    if not robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    print("SIM teleop started. Open the printed URL in the Quest browser, hold Move to drive.")
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
