#!/usr/bin/env python
"""Re-map ONE joint's range by hand-sweeping it, patching only that joint.

    uv run python calibrate_joint.py <arm> <joint> [seconds]

arm:   leader | follower
joint: shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper

Use when a single joint's calibrated range is narrower than its physical travel
(e.g. it "doesn't go all the way" one direction) but the rest of the arm is
fine -- avoids re-running full `lerobot-calibrate`, which would clobber the
other joints' (carefully tuned) ranges.

For the follower it first LOCKS every other joint in place (goal=present, torque
on -> no motion) so the arm holds its pose while you move just the target joint,
then frees the target (torque off) so you can sweep it by hand. It records the
target's `Present_Position` min/max, then patches that joint's `range_min`,
`range_max`, and its current hardware `homing_offset` (all in one frame) in the
arm's calibration JSON. Every other joint is left untouched.

Note: works for hand-movable joints. The follower GRIPPER won't back-drive by
hand -- use `find_follower_gripper.py` (motor-driven) for that one.
"""
import json
import os
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus

from so101_config import ARMS, SO101_MOTORS as JOINTS

MIN_SPAN = 100  # ticks; below this we assume the joint wasn't actually swept


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] not in ARMS or sys.argv[2] not in JOINTS:
        print(__doc__)
        return 2
    arm, joint = sys.argv[1], sys.argv[2]
    seconds = float(sys.argv[3]) if len(sys.argv) >= 4 else 15.0
    port = ARMS[arm]["port"]
    calib = os.path.expanduser(ARMS[arm]["calib"])
    if not os.path.exists(calib):
        print(f"ABORT: calibration file not found: {calib}")
        return 1

    bus = FeetechMotorsBus(port, JOINTS)
    bus._connect(handshake=False)
    bus.set_baudrate(1000000)

    # Hold every OTHER joint in place so the arm keeps its pose while you move
    # the target. Goal=present BEFORE enabling torque => no motion.
    if arm == "follower":
        for name in JOINTS:
            if name == joint:
                continue
            bus.write("Operating_Mode", name, 0, normalize=False)
            here = bus.read("Present_Position", name, normalize=False, num_retry=2)
            bus.write("Goal_Position", name, here, normalize=False)
            bus.write("Torque_Enable", name, 1, normalize=False)
        print(f">>> follower: all joints LOCKED except '{joint}' (which is free).")

    bus.disable_torque(joint)
    hw_home = bus.read("Homing_Offset", joint, normalize=False, num_retry=2)

    print(f"\n>>> {arm} '{joint}': MOVE IT BY HAND through its FULL range,")
    print(f">>> end to end, repeatedly, for the next {seconds:.0f} seconds ...\n")

    v = bus.read("Present_Position", joint, normalize=False, num_retry=2)
    mn = mx = v
    t_end = time.time() + seconds
    while time.time() < t_end:
        v = bus.read("Present_Position", joint, normalize=False, num_retry=2)
        mn, mx = min(mn, v), max(mx, v)
        print(f"\r  pos={v:5d}   min={mn:5d}   max={mx:5d}   span={mx - mn:5d}", end="", flush=True)
        time.sleep(0.02)
    print()
    bus.port_handler.closePort()

    span = mx - mn
    if span < MIN_SPAN:
        print(f"\nWARNING: span {span} < {MIN_SPAN}; not saving (joint barely moved).")
        return 1

    with open(calib) as f:
        cal = json.load(f)
    old = cal[joint].copy()
    cal[joint]["homing_offset"] = int(hw_home)  # keep range + homing in one frame
    cal[joint]["range_min"] = int(mn)
    cal[joint]["range_max"] = int(mx)
    with open(calib, "w") as f:
        json.dump(cal, f, indent=4)
        f.write("\n")

    print(f"\nSaved {calib}")
    print(f"  {joint} range_min: {old['range_min']} -> {int(mn)}")
    print(f"  {joint} range_max: {old['range_max']} -> {int(mx)}   (span {span})")
    print("Homing offset kept in-frame; all other joints untouched.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
