#!/usr/bin/env python
"""Trace a Cartesian toolpath on the SO-101 follower.

    waypoints (EE poses)  ->  placo inverse kinematics  ->  joint targets  ->  arm

    uv run python trace_path.py                 # DRY RUN: solve a demo path, report error
    uv run python trace_path.py --shape line    # demo: straight line sweep
    uv run python trace_path.py --csv out.csv   # also dump the joint trajectory
    uv run python trace_path.py --execute       # ALSO stream to the follower (ARM MOVES)

This is the first slice of the CAD -> path -> motion pipeline: it proves the
FK / IK / execute loop end to end. There is **no collision checking** here --
placo IK is local and geometry-unaware. Interference-free planning
(cuRobo / Tesseract-Descartes / MoveIt) bolts on later; see
docs/TOOLPATH_PLANNING.md.

Prereqs:
    ./setup.sh                      # installs placo + fetches the SO-101 URDF

Notes / caveats:
- The SO-101 is a 5-DOF arm. It cannot hold an arbitrary tool orientation
  everywhere; IK weights position over orientation and reports the orientation
  residual so you can see where the 5-DOF workspace runs out.
- Joint angles from IK are in the URDF's degree convention, which lerobot's own
  EE control treats as the robot's degree convention -- so we command them
  straight through as `<joint>.pos`.
"""

import argparse
import sys
import time

import numpy as np

from ik import load_kin, solve_ik
from so101_config import ARM_JOINTS, FOLLOWER_ID, FOLLOWER_PORT

# A comfortable, non-singular "ready" seed (degrees), arm reaching out front.
SEED_DEG = np.array([0.0, -35.0, 45.0, -10.0, 0.0, 0.0])


def _orient_err_deg(R_a, R_b):
    c = (np.trace(R_a.T @ R_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def make_path(kin, shape: str, size: float = 0.06, n: int = 40):
    """Build a list of EE pose waypoints (4x4) by moving the seed pose's
    position along a shape while holding its orientation."""
    anchor = kin.forward_kinematics(SEED_DEG).copy()
    R = anchor[:3, :3]
    c = anchor[:3, 3]
    poses = []
    if shape == "line":  # sweep along the arm's Y (side to side)
        for t in np.linspace(-0.5, 0.5, n):
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = c + np.array([0.0, t * size * 2, 0.0])
            poses.append(T)
    elif shape == "square":  # a box in the Y-Z plane at fixed X
        h = size / 2
        corners = [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]
        for (y0, z0), (y1, z1) in zip(corners[:-1], corners[1:]):
            for t in np.linspace(0, 1, max(2, n // 4), endpoint=False):
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = c + np.array([0.0, y0 + t * (y1 - y0), z0 + t * (z1 - z0)])
                poses.append(T)
    else:
        raise ValueError(f"unknown shape {shape!r} (use line|square)")
    return poses, anchor


def solve_trajectory(kin, poses, orientation_weight=0.0):
    """IK every waypoint, seeding each from the previous solution."""
    traj = []
    q = SEED_DEG.copy()
    for T in poses:
        q, perr = solve_ik(kin, q, T, orientation_weight=orientation_weight)
        oerr = _orient_err_deg(kin.forward_kinematics(q)[:3, :3], T[:3, :3])
        traj.append({"q": q.copy(), "pos_err_mm": perr, "orient_err_deg": oerr})
    return traj


def report(traj):
    perr = np.array([p["pos_err_mm"] for p in traj])
    oerr = np.array([p["orient_err_deg"] for p in traj])
    print(f"waypoints: {len(traj)}")
    print(f"position error  (mm) : mean {perr.mean():5.2f}  max {perr.max():5.2f}")
    print(f"orient.  error (deg) : mean {oerr.mean():5.2f}  max {oerr.max():5.2f}")
    bad = int((perr > 2.0).sum())
    if bad:
        print(f"WARNING: {bad} waypoint(s) with >2 mm position error "
              f"(outside reach, or raise --orient-weight is fighting position).")
    else:
        print("all waypoints reachable within 2 mm.")
    if oerr.max() > 3.0:
        print("note: orientation drifts because this is a 5-DOF arm holding position "
              "over a workspace -- fine for a spin/tilt-tolerant tool; raise "
              "--orient-weight to trade reach for tool orientation.")


def dump_csv(traj, path):
    import csv
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([*ARM_JOINTS, "gripper", "pos_err_mm", "orient_err_deg"])
        for p in traj:
            w.writerow([*np.round(p["q"], 4), round(p["pos_err_mm"], 3), round(p["orient_err_deg"], 3)])
    print(f"wrote {path}")


def execute(traj, hz=20.0):
    """Stream the joint trajectory to the follower. ARM MOVES."""
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower

    cfg = SOFollowerRobotConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID, max_relative_target=12.0)
    robot = SOFollower(cfg)
    robot.connect(calibrate=False)
    try:
        obs = robot.get_observation()
        gripper = obs.get("gripper.pos", 0.0)

        def arm_action(q):
            act = {f"{name}.pos": float(q[i]) for i, name in enumerate(ARM_JOINTS)}
            act["gripper.pos"] = float(gripper)  # hold gripper; IK doesn't own it
            return act

        # Ease from the current pose to the first waypoint over ~1.5 s.
        start = np.array([obs[f"{n}.pos"] for n in ARM_JOINTS])
        goal = traj[0]["q"][: len(ARM_JOINTS)]
        for a in np.linspace(0, 1, 30):
            robot.send_action(arm_action((1 - a) * start + a * goal))
            time.sleep(1.5 / 30)

        dt = 1.0 / hz
        for p in traj:
            robot.send_action(arm_action(p["q"]))
            time.sleep(dt)
        print("path complete.")
    finally:
        try:
            robot.bus.disable_torque()
            robot.disconnect()
        except Exception:
            pass
        print("torque released, disconnected.")


def main() -> int:
    ap = argparse.ArgumentParser(description="Trace a Cartesian toolpath on the SO-101 follower.")
    ap.add_argument("--shape", choices=["line", "square"], default="square")
    ap.add_argument("--size", type=float, default=0.06, help="path size in meters")
    ap.add_argument("--points", type=int, default=40)
    ap.add_argument("--csv", metavar="FILE", help="dump joint trajectory to CSV")
    ap.add_argument("--orient-weight", type=float, default=0.0,
                    help="IK orientation weight (0 = position-only; higher = hold tool "
                         "orientation at the cost of position/reach on this 5-DOF arm)")
    ap.add_argument("--execute", action="store_true", help="stream to the follower (ARM MOVES)")
    ap.add_argument("--hz", type=float, default=20.0)
    args = ap.parse_args()

    try:
        kin = load_kin()
    except Exception as e:
        print(f"Could not load kinematics: {type(e).__name__}: {e}")
        print("Run ./setup.sh (installs placo + fetches the URDF).")
        return 1

    poses, _ = make_path(kin, args.shape, args.size, args.points)
    traj = solve_trajectory(kin, poses, orientation_weight=args.orient_weight)
    report(traj)
    if args.csv:
        dump_csv(traj, args.csv)

    if args.execute:
        print("\n--execute: the arm will now move through the path. Ctrl-C to abort.")
        execute(traj, hz=args.hz)
    else:
        print("\n(dry run -- no motion. Re-run with --execute to drive the arm.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
