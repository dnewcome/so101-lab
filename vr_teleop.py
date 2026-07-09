"""Bimanual VR teleop for the SO-101.

    controller pose --clutch--> EE target --IK--> joint targets --> arm(s)

    uv run python vr_teleop.py                    # SIM backend + MOCK controllers (no headset)
    uv run python vr_teleop.py --view             # + MuJoCo viewer window
    uv run python vr_teleop.py --source oculus     # real Quest via oculus_reader (sideloaded APK)
    uv run python vr_teleop.py --backend hardware  # drive two real SOFollowers
    uv run python vr_teleop.py --source oculus --rerun   # headset drives sim; watch latency + tracking
    uv run python vr_teleop.py --iters 240        # headless: prints per-tick latency (no headset)

Knobs that make it feel right (tune with the headset on, watch --rerun):
  --axis-map  remap the Quest tracking frame to the robot base ("x=-z,y=-x,z=y").
              Flip a sign if moving a hand drives the EE the wrong way.
  --pos-scale controller-delta -> EE-delta gain (1.0 = 1:1; drop below 1 for finer control).
  --max-step  spike limiter: hard cap on EE motion per frame (m). A wild tracking
              frame can't jump the arm further than this; huge single-frame
              outliers are dropped entirely. Lower = calmer but laggier.
  --smooth    1.0 = pure rate limit; <1 (e.g. 0.5) adds extra low-pass damping.

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

AXES = {"x": 0, "y": 1, "z": 2}
# Maps each ROBOT-BASE axis to a signed QUEST-tracking axis. Default guess:
# Quest (right, up, back) -> SO-101 base (forward, left, up). It is a STARTING
# POINT, not gospel — put the headset on and flip a sign here (or via --axis-map)
# if a hand motion drives the EE the wrong way. The Rerun EE plot (--rerun) makes
# a wrong axis obvious: the commanded point moves off the axis your hand did.
DEFAULT_AXIS_MAP = "x=-z,y=-x,z=y"


def parse_axis_map(spec: str) -> np.ndarray:
    """Compact axis spec -> 3x3 R with base_delta = R @ quest_delta.

    ``"x=-z,y=-x,z=y"`` reads as: base +X follows Quest -Z, base +Y follows
    Quest -X, base +Z follows Quest +Y. Lets you retune the Quest->robot frame
    between runs without touching code — just flip a sign.
    """
    R = np.zeros((3, 3))
    for entry in spec.split(","):
        base_ax, src = entry.split("=")
        src = src.strip().lower()
        sign = -1.0 if src[0] == "-" else 1.0
        R[AXES[base_ax.strip().lower()], AXES[src[-1]]] = sign
    if not (np.allclose(np.abs(R).sum(0), 1) and np.allclose(np.abs(R).sum(1), 1)):
        raise ValueError(f"axis map {spec!r} is not a signed permutation of x,y,z")
    return R


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
    # A frame whose step exceeds REJECT_MULT * max_step is "wild". A wild frame is
    # HELD (dropped) for up to REJECT_MAX consecutive frames — that kills transient
    # single-frame glitches. If it stays wild past REJECT_MAX, it's a real
    # relocation, so we slew toward it at max_step/frame instead of freezing.
    REJECT_MULT = 6.0
    REJECT_MAX = 4

    def __init__(self, kin, seed=SEED, pos_scale=1.0, orientation_weight=0.0, axis_R=None,
                 max_step=0.02, smooth=1.0):
        self.kin = kin
        self.q = np.array(seed, dtype=float)
        self.pos_scale = pos_scale
        self.orientation_weight = orientation_weight
        self.axis_R = np.eye(3) if axis_R is None else np.asarray(axis_R, dtype=float)
        self.max_step = max_step   # hard cap on EE move per frame (m) — the spike limiter
        self.smooth = smooth       # 1.0 = pure rate limit; <1 adds extra damping
        self.ref_ee = None
        self.ref_ctrl = None
        self.ee = None             # committed (rate-limited) EE target position
        self.reject_run = 0
        self.raw_step_mm = 0.0     # telemetry: pre-guard commanded step
        self.guard = 0             # telemetry: 0 pass-through, 1 rate-limited, 2 rejected

    def update(self, cs: ControllerState):
        if cs.grip:
            if self.ref_ee is None:  # latch reference on grip press
                self.ref_ee = self.kin.forward_kinematics(self.q).copy()
                self.ref_ctrl = cs.pose.copy()
                self.ee = self.ref_ee[:3, 3].copy()
                self.reject_run = 0
            # Rotate the raw controller-frame delta into the robot base frame,
            # THEN scale. Without axis_R, "hand right" rarely means "EE right".
            dp = self.axis_R @ (cs.pose[:3, 3] - self.ref_ctrl[:3, 3]) * self.pos_scale
            step = (self.ref_ee[:3, 3] + dp) - self.ee  # move from committed EE toward target
            dist = float(np.linalg.norm(step))
            self.raw_step_mm = dist * 1000.0
            if dist > self.REJECT_MULT * self.max_step:
                if self.reject_run < self.REJECT_MAX:
                    self.reject_run += 1  # transient wild frame -> hold (glitch)
                    self.guard = 2
                    step[:] = 0.0
                else:  # stayed wild -> real relocation: slew in, don't freeze
                    step *= self.max_step / dist
                    self.guard = 1
            else:
                self.reject_run = 0
                if dist > self.max_step:  # too fast -> slew-clamp to max_step
                    step *= self.max_step / dist
                    self.guard = 1
                else:
                    self.guard = 0
            self.ee = self.ee + self.smooth * step
            T = self.ref_ee.copy()
            T[:3, 3] = self.ee
            self.q, _ = solve_ik(self.kin, self.q, T, orientation_weight=self.orientation_weight)
        else:
            self.ref_ee = None
            self.ref_ctrl = None
            self.reject_run = 0
            self.guard = 0
            self.raw_step_mm = 0.0
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
def _log_side(rr, side, cmd, achieved, cs):
    """Log one hand's commanded/achieved EE + inputs for this tick."""
    rr.log(f"teleop/{side}/cmd_ee", rr.Points3D([cmd], radii=0.006, colors=[0, 190, 255]))
    rr.log(f"teleop/{side}/clutch", rr.Scalars(1.0 if cs.grip else 0.0))
    rr.log(f"teleop/{side}/trigger", rr.Scalars(float(cs.trigger)))
    if achieved is not None:  # sim backend can read the arm back; hardware can't (yet)
        rr.log(f"teleop/{side}/achieved_ee", rr.Points3D([achieved], radii=0.006, colors=[255, 120, 0]))
        rr.log(f"teleop/{side}/track_err_mm", rr.Scalars(float(np.linalg.norm(cmd - achieved) * 1000.0)))


def run(source, backend, kin, iters=None, hz=30.0, pos_scale=1.0, axis_R=None, log=False,
        max_step=0.02, smooth=1.0):
    arms = {s: ClutchArm(kin, pos_scale=pos_scale, axis_R=axis_R,
                         max_step=max_step, smooth=smooth) for s in SIDES}
    ee_lo = {s: np.full(3, np.inf) for s in SIDES}
    ee_hi = {s: np.full(3, -np.inf) for s in SIDES}
    dt = 1.0 / hz

    rr = None
    if log:
        from lerobot.utils.visualization_utils import init_rerun
        import rerun as rr  # noqa: local handle; passed to _log_side
        init_rerun(session_name="so101_vr_teleop")

    compute_ms = []  # pose-read + IK + command latency per tick (the teleop-feel number)
    loop_hz = []
    t_prev = None
    k = 0
    try:
        while iters is None or k < iters:
            t_top = time.perf_counter()
            states = source.poses()
            if rr is not None:
                rr.set_time("tick", sequence=k)
            for side in SIDES:
                if side not in states:
                    continue
                q, trig = arms[side].update(states[side])
                backend.command(side, q, trig)
                cmd = kin.forward_kinematics(q)[:3, 3]
                ee_lo[side] = np.minimum(ee_lo[side], cmd)
                ee_hi[side] = np.maximum(ee_hi[side], cmd)
                if rr is not None:
                    achieved = backend.ee(side, kin) if hasattr(backend, "ee") else None
                    _log_side(rr, side, cmd, achieved, states[side])
                    rr.log(f"teleop/{side}/raw_step_mm", rr.Scalars(arms[side].raw_step_mm))
                    rr.log(f"teleop/{side}/guard", rr.Scalars(float(arms[side].guard)))
            t_cmd = time.perf_counter()
            backend.tick()  # sim physics / no-op on hardware — excluded from latency

            compute_ms.append((t_cmd - t_top) * 1000.0)
            if t_prev is not None:
                loop_hz.append(1.0 / max(t_top - t_prev, 1e-6))
            t_prev = t_top
            if rr is not None:
                rr.log("teleop/timing/compute_ms", rr.Scalars(compute_ms[-1]))
                if loop_hz:
                    rr.log("teleop/timing/loop_hz", rr.Scalars(loop_hz[-1]))
            time.sleep(dt if iters is None else 0.0)
            k += 1
    finally:
        backend.close()

    extent = {s: (ee_hi[s] - ee_lo[s]) for s in SIDES}  # EE travel extent per arm
    timing = {
        "compute_ms_mean": float(np.mean(compute_ms)) if compute_ms else 0.0,
        "compute_ms_max": float(np.max(compute_ms)) if compute_ms else 0.0,
        "loop_hz_mean": float(np.mean(loop_hz)) if loop_hz else 0.0,
    }
    return extent, timing


def main() -> int:
    ap = argparse.ArgumentParser(description="Bimanual VR teleop for the SO-101.")
    ap.add_argument("--source", choices=["mock", "oculus"], default="mock")
    ap.add_argument("--backend", choices=["sim", "hardware"], default="sim")
    ap.add_argument("--view", action="store_true", help="MuJoCo viewer (sim only)")
    ap.add_argument("--iters", type=int, default=None, help="stop after N steps (default: run forever)")
    ap.add_argument("--pos-scale", type=float, default=1.0,
                    help="controller-delta -> EE-delta gain (1.0 = 1:1); <1 for finer control")
    ap.add_argument("--axis-map", default=DEFAULT_AXIS_MAP,
                    help=f"Quest->base axis remap, e.g. {DEFAULT_AXIS_MAP!r}; flip a sign if a hand moves the EE wrong")
    ap.add_argument("--max-step", type=float, default=0.02,
                    help="spike limiter: max EE move per frame in metres (lower = calmer, more lag)")
    ap.add_argument("--smooth", type=float, default=1.0,
                    help="1.0 = pure rate limit; <1 adds extra damping (e.g. 0.5)")
    ap.add_argument("--rerun", action="store_true",
                    help="log latency + commanded-vs-achieved EE + clutch/trigger + spike guard to Rerun")
    args = ap.parse_args()

    try:
        axis_R = parse_axis_map(args.axis_map)
    except (ValueError, KeyError) as e:
        print(f"bad --axis-map {args.axis_map!r}: {e}")
        return 1

    try:
        kin = load_kin()
    except Exception as e:
        print(f"kinematics unavailable: {type(e).__name__}: {e}")
        print("Run ./setup.sh (installs placo + fetches the URDF).")
        return 1

    if args.source == "mock":
        source = MockSource()
    else:
        try:
            source = OculusSource()
        except ModuleNotFoundError:
            print("oculus_reader not installed (the native Quest driver). Install it:")
            print('  uv pip install "oculus_reader @ git+https://github.com/rail-berkeley/oculus_reader.git"')
            return 1
        except Exception as e:
            print(f"couldn't reach the Quest via oculus_reader: {type(e).__name__}: {e}")
            print("Check: headset plugged in, Developer Mode on, ADB authorized (accept the")
            print("in-headset prompt), then `adb devices` should list it. oculus_reader")
            print("auto-installs its APK once the device is reachable.")
            return 1

    backend = SimBackend(view=args.view) if args.backend == "sim" else HardwareBackend()

    if args.iters:
        extent, timing = run(source, backend, kin, iters=args.iters,
                             pos_scale=args.pos_scale, axis_R=axis_R, log=args.rerun,
                             max_step=args.max_step, smooth=args.smooth)
        for s in SIDES:
            print(f"{s:5s} arm EE travel (m): {np.round(extent[s], 3)}")
        print(f"latency/tick: mean {timing['compute_ms_mean']:.2f} ms  "
              f"max {timing['compute_ms_max']:.2f} ms   throughput {timing['loop_hz_mean']:.0f} Hz")
        print("both hands drove their arms." if all(extent[s].max() > 0.01 for s in SIDES)
              else "WARNING: arms barely moved.")
    else:
        print(f"running: source={args.source} backend={args.backend} "
              f"pos_scale={args.pos_scale} axis_map={args.axis_map!r} "
              f"max_step={args.max_step} smooth={args.smooth}"
              f"{' +rerun' if args.rerun else ''}. Ctrl-C to stop.")
        run(source, backend, kin, pos_scale=args.pos_scale, axis_R=axis_R, log=args.rerun,
            max_step=args.max_step, smooth=args.smooth)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
