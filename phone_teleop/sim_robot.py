"""A single SO-101 in MuJoCo that quacks like a lerobot `SOFollower`.

Just enough of the robot interface for the phone/EE pipeline to drive it:
`bus.motors.keys()`, `get_observation()` -> {"<joint>.pos": deg},
`send_action({"<joint>.pos": deg})`, `connect()/is_connected/disconnect()`.
So `teleoperate_sim.py` runs the *exact* lerobot phone pipeline, only the last
mile (serial → MuJoCo actuators) changes.

    uv run python phone_teleop/sim_robot.py --view   # self-test: canned motion
"""

import os
import sys
from collections import OrderedDict
from types import SimpleNamespace

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from so101_config import MJCF_SCENE, SO101_MOTORS  # noqa: E402

JOINT_ORDER = list(SO101_MOTORS.keys())  # shoulder_pan..gripper (URDF/MJCF order)
SEED_DEG = np.array([0.0, -35.0, 45.0, -10.0, 0.0, 0.0])


class SimRobot:
    name = "so101_follower_sim"

    def __init__(self, mjcf: str = MJCF_SCENE, view: bool = False, render: bool = False,
                 settle_steps: int = 400, res=(480, 640)):
        if mujoco is None:
            raise ImportError("mujoco not installed. Run: uv sync --extra sim")
        self.model = mujoco.MjModel.from_xml_path(mjcf)
        self.data = mujoco.MjData(self.model)
        self._render = render
        self._res = res
        self.renderer = None
        self._cam = None
        # motors dict — order is what the pipeline uses for joint_names + IK output
        self.bus = SimpleNamespace(motors=OrderedDict((j, None) for j in JOINT_ORDER))
        self._act = {j: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in JOINT_ORDER}
        self._qadr = {
            j: self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in JOINT_ORDER
        }
        self._view = view
        self._settle = settle_steps
        self.viewer = None
        self._connected = False

    def connect(self, calibrate: bool = False) -> None:
        for i, j in enumerate(JOINT_ORDER):
            self.data.ctrl[self._act[j]] = np.deg2rad(SEED_DEG[i])
        mujoco.mj_step(self.model, self.data, self._settle)
        if self._view:
            import mujoco.viewer as _mjviewer  # aliased so it doesn't shadow global `mujoco`
            self.viewer = _mjviewer.launch_passive(self.model, self.data)
        if self._render:
            os.environ.setdefault("MUJOCO_GL", "egl")  # headless offscreen GL
            self.renderer = mujoco.Renderer(self.model, self._res[0], self._res[1])
            self._cam = mujoco.MjvCamera()
            self._cam.distance, self._cam.azimuth, self._cam.elevation = 0.9, 130, -20
            self._cam.lookat[:] = [0.15, 0.0, 0.15]
        self._connected = True

    def frame(self):
        """Offscreen RGB render of the current scene (or None if render=False)."""
        if self.renderer is None:
            return None
        self.renderer.update_scene(self.data, self._cam)
        return self.renderer.render()

    @property
    def is_connected(self) -> bool:
        return self._connected

    def get_observation(self) -> dict:
        return {f"{j}.pos": float(np.rad2deg(self.data.qpos[self._qadr[j]])) for j in JOINT_ORDER}

    def send_action(self, action: dict) -> dict:
        for j in JOINT_ORDER:
            key = f"{j}.pos"
            if key in action:
                self.data.ctrl[self._act[j]] = np.deg2rad(float(action[key]))
        mujoco.mj_step(self.model, self.data, 8)
        if self.viewer is not None:
            self.viewer.sync()
        return action

    def disconnect(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        self._connected = False


def _self_test(view: bool) -> int:
    robot = SimRobot(view=view)
    robot.connect()
    import time as _t
    start = robot.get_observation()
    print("observation keys:", list(start))
    # sweep shoulder_pan +-25 deg and confirm it tracks
    max_err = 0.0
    for k in range(120):
        target = dict(start)
        target["shoulder_pan.pos"] = 25.0 * np.sin(2 * np.pi * k / 120)
        robot.send_action(target)
        err = abs(robot.get_observation()["shoulder_pan.pos"] - target["shoulder_pan.pos"])
        max_err = max(max_err, err)
        if not view:
            continue
        _t.sleep(1 / 60)
    robot.disconnect()
    print(f"sim robot drove; max shoulder_pan track err: {max_err:.2f} deg  ->", "OK" if max_err < 5 else "CHECK")
    return 0


if __name__ == "__main__":
    raise SystemExit(_self_test(view="--view" in sys.argv))
