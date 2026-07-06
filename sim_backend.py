"""Bimanual SO-101 in MuJoCo — the simulation backend for teleop/toolpaths.

Loads two copies of the SO-ARM100 MJCF into one scene (left_/right_ prefixes,
mirrored bases) and lets you command each arm's joints. It's a drop-in
alternative to driving real `SOFollower`s: same joint targets (degrees), just
set into MuJoCo actuators instead of over serial.

    ./fetch_urdf.sh          # also drops so101_new_calib.xml + meshes in ./urdf/
    uv sync --extra sim      # mujoco

    uv run python sim_backend.py            # self-test: sweep both arms, report
    uv run python sim_backend.py --view     # open the MuJoCo viewer (needs a display)
"""

import sys

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

from so101_config import ARM_JOINTS, MJCF_PATH

SIDES = ("left", "right")


class BimanualSim:
    """Two SO-101 arms in one MuJoCo scene, commanded by joint angle (degrees)."""

    def __init__(self, mjcf: str = MJCF_PATH, separation: float = 0.35):
        if mujoco is None:
            raise ImportError("mujoco not installed. Run: uv sync --extra sim")
        spec = mujoco.MjSpec()
        for side, y in (("left", separation / 2), ("right", -separation / 2)):
            child = mujoco.MjSpec.from_file(mjcf)
            frame = child.worldbody.first_body()
            f = spec.worldbody.add_frame()
            f.pos = [0.0, y, 0.0]
            f.attach_body(frame, f"{side}_", "")
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        # Resolve actuator ids for the 5 arm joints + gripper, per side.
        self._act = {}
        for side in SIDES:
            for j in (*ARM_JOINTS, "gripper"):
                name = f"{side}_{j}"
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self._act[(side, j)] = aid
        self._qadr = {}
        for side in SIDES:
            for j in (*ARM_JOINTS, "gripper"):
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{j}")
                self._qadr[(side, j)] = self.model.jnt_qposadr[jid]
        mujoco.mj_forward(self.model, self.data)

    def set_arm_targets(self, side: str, q_deg) -> None:
        """Command the 5 arm joints of `side` (list/array of degrees)."""
        for i, j in enumerate(ARM_JOINTS):
            self.data.ctrl[self._act[(side, j)]] = np.deg2rad(q_deg[i])

    def set_gripper(self, side: str, deg: float) -> None:
        self.data.ctrl[self._act[(side, "gripper")]] = np.deg2rad(deg)

    def get_arm_q(self, side: str) -> np.ndarray:
        """Current joint angles (degrees) of the 5 arm joints of `side`."""
        return np.rad2deg([self.data.qpos[self._qadr[(side, j)]] for j in ARM_JOINTS])

    def step(self, n: int = 1) -> None:
        mujoco.mj_step(self.model, self.data, n)

    def launch_viewer(self):
        import mujoco.viewer
        return mujoco.viewer.launch_passive(self.model, self.data)


def _self_test(view: bool = False) -> int:
    from ik import load_kin, solve_ik

    sim = BimanualSim()
    kin = load_kin()
    seed = np.array([0.0, -35.0, 45.0, -10.0, 0.0, 0.0])

    # Command each arm along a small vertical sweep and check it tracks.
    anchor = kin.forward_kinematics(seed).copy()
    viewer = sim.launch_viewer() if view else None
    # Settle at the seed first so we measure steady-state tracking, not startup.
    for side in SIDES:
        sim.set_arm_targets(side, seed[:5])
    sim.step(500)
    max_err = 0.0
    q = seed.copy()
    try:
        for k in range(150):
            z = 0.04 * np.sin(2 * np.pi * k / 150)  # slow vertical sweep
            T = anchor.copy()
            T[2, 3] += z
            q, _ = solve_ik(kin, q, T)
            for side in SIDES:
                sim.set_arm_targets(side, q[:5])
            sim.step(40)  # settle at a realistic control rate (~human speed)
            # tracking error of the LEFT arm's commanded vs achieved joints
            err = np.linalg.norm(sim.get_arm_q("left") - q[:5])
            max_err = max(max_err, err)
            if viewer is not None:
                viewer.sync()
    finally:
        if viewer is not None:
            viewer.close()
    print(f"combined model: {sim.model.njnt} joints, {sim.model.nu} actuators")
    print(f"both arms driven; max |commanded-achieved| (left): {max_err:.2f} deg")
    print("OK" if max_err < 5.0 else "WARNING: arms not tracking targets (actuator tuning?)")
    return 0


if __name__ == "__main__":
    raise SystemExit(_self_test(view="--view" in sys.argv))
