"""Machine + arm configuration for the SO-101 tools.

EDIT the serials / ids below for YOUR setup (or set the matching env vars).
Find your controller-board serials with:

    ls /dev/serial/by-id/

Both SO-101 boards use the same CH9102 USB chip and the same USB VID:PID, so
`/dev/ttyACM*` numbering can swap across reboots. The `by-id` names are keyed to
each board's unique serial number, so they're stable — always address the arms
by those, never by `ttyACMx`.
"""

import os

from lerobot.motors import Motor, MotorNormMode


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


# --- Serial ports (stable /dev/serial/by-id/ names; unique per board) ---------
LEADER_PORT = _env(
    "SO101_LEADER_PORT",
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D049885-if00",
)
FOLLOWER_PORT = _env(
    "SO101_FOLLOWER_PORT",
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61033654-if00",
)

# --- lerobot calibration ids --------------------------------------------------
LEADER_ID = _env("SO101_LEADER_ID", "my_so101_leader")
FOLLOWER_ID = _env("SO101_FOLLOWER_ID", "my_so101")

# lerobot stores calibration JSON under CALIB_DIR/<kind>/<class>/<id>.json
CALIB_DIR = os.path.expanduser(
    _env("SO101_CALIB_DIR", "~/.cache/huggingface/lerobot/calibration")
)
LEADER_CALIB = os.path.join(CALIB_DIR, "teleoperators", "so_leader", f"{LEADER_ID}.json")
FOLLOWER_CALIB = os.path.join(CALIB_DIR, "robots", "so_follower", f"{FOLLOWER_ID}.json")

ARMS = {
    "leader": {"port": LEADER_PORT, "calib": LEADER_CALIB, "id": LEADER_ID},
    "follower": {"port": FOLLOWER_PORT, "calib": FOLLOWER_CALIB, "id": FOLLOWER_ID},
}

# --- Kinematics (URDF for placo-based FK/IK) ----------------------------------
# The SO-101 URDF + meshes come from TheRobotStudio/SO-ARM100 (Apache-2.0).
# Run `./fetch_urdf.sh` to populate ./urdf/ (gitignored), or point SO101_URDF at
# your own copy. `gripper_frame_link` is the end-effector frame lerobot uses.
_HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.expanduser(
    _env("SO101_URDF", os.path.join(_HERE, "urdf", "so101_new_calib.urdf"))
)
URDF_TARGET_FRAME = _env("SO101_URDF_FRAME", "gripper_frame_link")

# MuJoCo MJCF model (ships alongside the URDF in SO-ARM100/Simulation/SO101).
# `./fetch_urdf.sh` drops it in ./urdf/ next to the .urdf.
MJCF_PATH = os.path.expanduser(
    _env("SO101_MJCF", os.path.join(_HERE, "urdf", "so101_new_calib.xml"))
)

# The five positioning joints (the gripper is the 6th servo but not a pose DOF).
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# --- SO-101 joint -> Feetech motor map (identical for follower and leader) -----
SO101_MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}
