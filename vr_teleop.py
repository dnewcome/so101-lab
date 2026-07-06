"""Bimanual VR teleop for the SO-101.

    controller pose --clutch--> EE target --IK--> joint targets --> arm(s)

    uv run python vr_teleop.py                    # SIM backend + MOCK controllers (no headset)
    uv run python vr_teleop.py --view             # + MuJoCo viewer window
    uv run python vr_teleop.py --source oculus     # real Quest via oculus_reader (sideloaded APK)
    uv run python vr_teleop.py --backend hardware  # drive two real SOFollowers

Two things are pluggable; the clutch + IK in the middle never change:
  - POSE SOURCE : mock (scripted) | oculus (Quest controllers over ADB/Wi-Fi)
  - BACKEND     : sim (MuJoCo, two arms) | hardware (two SOFollowers)

Clutch model (per hand, like lerobot's EEReferenceAndDelta): while you hold the
GRIP button, the controller's motion *delta* maps to an end-effector *delta*
(so you don't need to calibrate the room-to-robot frame). Release to freeze.
The TRIGGER drives the gripper. SO-101 is 5-DOF, so this is position-primary.
"""

import argparse
import time
from dataclasses import dataclass

import numpy as np

from ik import load_kin, solve_ik
from so101_config import ARM_JOINTS

SIDES = ("left", "right")
SEED = np.array([0.0, -35.0, 45.0, -10.0, 0.0, 0.0])


@dataclass
class ControllerState:
    pose: np.ndarray  # 4x4 in the tracking frame
    grip: bool        # clutch engaged
    trigger: float    # 0..1 -> gripper


# --------------------------------------------------------------------------- #
# Pose sources
# --------------------------------------------------------------------------- #
class MockSource:
    """Scripted controllers (no headset): each hand traces a small circle with
    the grip held, trigger cycling. Lets you exercise the whole loop in sim."""

    def __init__(self):
        self.k = 0

    def poses(self):
        self.k += 1
        out = {}
        for side, phase in (("left", 0.0), ("right", np.pi)):
            ang = 2 * np.pi * self.k / 160 + phase
            T = np.eye(4)
            T[:3, 3] = [0.0, 0.04 * np.cos(ang), 0.04 * np.sin(ang)]  # 4 cm circle, Y-Z
            trig = 0.5 + 0.5 * np.sin(2 * np.pi * self.k / 80)
            out[side] = ControllerState(pose=T, grip=True, trigger=trig)
        return out


class OculusSource:
    """Quest controllers via `oculus_reader` (sideload its APK, connect over
    ADB/Wi-Fi). Maps its transforms + buttons to ControllerState.

        pip install oculus-reader   # plus the APK on the headset
    """

    def __init__(self):
        from oculus_reader.reader import OculusReader  # noqa: lazy import
        self.reader = OculusReader()

    def poses(self):
        transforms, buttons = self.reader.get_transformations_and_buttons()
        out = {}
        for side, key in (("left", "l"), ("right", "r")):
            if key not in transforms:
                continue
            out[side] = ControllerState(
                pose=np.asarray(transforms[key]),
                grip=bool(buttons.get(f"{key.upper()}G", False)),  # grip button
                trigger=float(buttons.get(f"{key.upper()}Tr", 0.0)),  # trigger analog
            )
        return out


# --------------------------------------------------------------------------- #
# Per-hand clutch + IK
# --------------------------------------------------------------------------- #
class ClutchArm:
    def __init__(self, kin, seed=SEED, pos_scale=1.0, orientation_weight=0.0):
        self.kin = kin
        self.q = np.array(seed, dtype=float)
        self.pos_scale = pos_scale
        self.orientation_weight = orientation_weight
        self.ref_ee = None
        self.ref_ctrl = None

    def update(self, cs: ControllerState):
        if cs.grip:
            if self.ref_ee is None:  # latch reference on grip press
                self.ref_ee = self.kin.forward_kinematics(self.q).copy()
                self.ref_ctrl = cs.pose.copy()
            dp = (cs.pose[:3, 3] - self.ref_ctrl[:3, 3]) * self.pos_scale
            T = self.ref_ee.copy()
            T[:3, 3] = self.ref_ee[:3, 3] + dp
            self.q, _ = solve_ik(self.kin, self.q, T, orientation_weight=self.orientation_weight)
        else:
            self.ref_ee = None
            self.ref_ctrl = None
        return self.q, cs.trigger


# --------------------------------------------------------------------------- #
# Backends
# --------------------------------------------------------------------------- #
class SimBackend:
    def __init__(self, view=False):
        from sim_backend import BimanualSim
        self.sim = BimanualSim()
        for s in SIDES:
            self.sim.set_arm_targets(s, SEED[:5])
        self.sim.step(500)  # settle at seed
        self.viewer = self.sim.launch_viewer() if view else None

    def command(self, side, q, trigger):
        self.sim.set_arm_targets(side, q[:5])
        self.sim.set_gripper(side, -10.0 + trigger * 110.0)  # map 0..1 to jaw range (deg)

    def tick(self):
        self.sim.step(8)
        if self.viewer is not None:
            self.viewer.sync()

    def ee(self, side, kin):  # for the self-test only
        return kin.forward_kinematics(np.append(self.sim.get_arm_q(side), 0.0))[:3, 3]

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


class HardwareBackend:
    """Two real SOFollowers keyed 'left'/'right'. Fill ARM_TABLE in multiarm.py."""

    def __init__(self, view=False):
        from multiarm import MultiArm
        self.ma = MultiArm()
        self.ma.connect(calibrate=False)

    def command(self, side, q, trigger):
        if side in self.ma.arms:
            act = {f"{n}.pos": float(q[i]) for i, n in enumerate(ARM_JOINTS)}
            act["gripper.pos"] = float(np.clip(trigger * 100.0, 0, 100))  # 0..100 %
            self.ma.arms[side].send_action(act)

    def tick(self):
        pass

    def close(self):
        self.ma.disconnect()


# --------------------------------------------------------------------------- #
def run(source, backend, kin, iters=None, hz=30.0):
    arms = {s: ClutchArm(kin) for s in SIDES}
    ee_lo = {s: np.full(3, np.inf) for s in SIDES}
    ee_hi = {s: np.full(3, -np.inf) for s in SIDES}
    dt = 1.0 / hz
    k = 0
    try:
        while iters is None or k < iters:
            states = source.poses()
            for side in SIDES:
                if side not in states:
                    continue
                q, trig = arms[side].update(states[side])
                backend.command(side, q, trig)
                p = kin.forward_kinematics(q)[:3, 3]
                ee_lo[side] = np.minimum(ee_lo[side], p)
                ee_hi[side] = np.maximum(ee_hi[side], p)
            backend.tick()
            time.sleep(dt if iters is None else 0.0)
            k += 1
    finally:
        backend.close()
    return {s: (ee_hi[s] - ee_lo[s]) for s in SIDES}  # EE travel extent per arm


def main() -> int:
    ap = argparse.ArgumentParser(description="Bimanual VR teleop for the SO-101.")
    ap.add_argument("--source", choices=["mock", "oculus"], default="mock")
    ap.add_argument("--backend", choices=["sim", "hardware"], default="sim")
    ap.add_argument("--view", action="store_true", help="MuJoCo viewer (sim only)")
    ap.add_argument("--iters", type=int, default=None, help="stop after N steps (default: run forever)")
    args = ap.parse_args()

    try:
        kin = load_kin()
    except Exception as e:
        print(f"kinematics unavailable: {type(e).__name__}: {e}")
        print("Run ./fetch_urdf.sh and `uv sync --extra kin`.")
        return 1

    source = MockSource() if args.source == "mock" else OculusSource()
    backend = SimBackend(view=args.view) if args.backend == "sim" else HardwareBackend()

    if args.iters:
        extent = run(source, backend, kin, iters=args.iters)
        for s in SIDES:
            print(f"{s:5s} arm EE travel (m): {np.round(extent[s], 3)}")
        print("both hands drove their arms." if all(extent[s].max() > 0.01 for s in SIDES)
              else "WARNING: arms barely moved.")
    else:
        print(f"running: source={args.source} backend={args.backend}. Ctrl-C to stop.")
        run(source, backend, kin)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
