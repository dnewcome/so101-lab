#!/usr/bin/env python
"""Re-calibrate ONLY the gripper's range on one SO-101 arm.

    uv run python calibrate_gripper.py [leader|follower] [seconds]

Use this when the other five joints are calibrated fine but the gripper is
jumpy in teleop because its recorded range is degenerate (e.g. a 1-tick span
because the trigger wasn't swept during `lerobot-calibrate`).

It reads the gripper's raw `Present_Position` while you work the trigger
through its FULL travel, records the min/max, and patches just
`gripper.range_min` / `gripper.range_max` in that arm's calibration JSON.
Everything else (homing offset, the five arm joints) is left untouched.

Good for the LEADER (spring trigger -- squeeze it during the capture). The
FOLLOWER gripper is too geared to back-drive by hand; use
`find_follower_gripper.py` (motor-driven) for that one.

Why this is safe / correct:
- Normalization uses the JSON range_min/max on the *raw* Present_Position
  (motors_bus `_normalize`), and Present_Position already includes the
  hardware Homing_Offset. We re-measure in that same frame, so the homing
  offset stays valid and only the range needs updating.
"""

import json
import os
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus

from so101_config import ARMS, SO101_MOTORS as JOINTS

MIN_HEALTHY_SPAN = 100  # ticks; below this the gripper will still be jumpy


def main() -> int:
    arm = sys.argv[1] if len(sys.argv) >= 2 else "leader"
    seconds = float(sys.argv[2]) if len(sys.argv) >= 3 else 12.0
    if arm not in ARMS:
        print(__doc__)
        print(f"valid arms: {', '.join(ARMS)}")
        return 2

    port = ARMS[arm]["port"]
    calib = os.path.expanduser(ARMS[arm]["calib"])
    if not os.path.exists(calib):
        print(f"ABORT: calibration file not found: {calib}")
        return 1

    bus = FeetechMotorsBus(port, JOINTS)
    bus._connect(handshake=False)
    bus.set_baudrate(1000000)

    # For the follower, hold the five arm joints rigid so both hands are free to
    # work the gripper (the arm often goes limp when teleop stops). Goal is set to
    # the present position BEFORE enabling torque, so locking causes NO motion.
    if arm == "follower":
        for name in JOINTS:
            if name == "gripper":
                continue
            bus.write("Operating_Mode", name, 0, normalize=False)  # position mode
            here = bus.read("Present_Position", name, normalize=False, num_retry=2)
            bus.write("Goal_Position", name, here, normalize=False)
            bus.write("Torque_Enable", name, 1, normalize=False)
        print(">>> follower arm joints LOCKED in place (gripper stays free).")

    # Free the gripper so it can be moved by hand (leader is already torque-off;
    # the follower needs this to be back-drivable).
    bus.disable_torque("gripper")

    print(f"\n>>> {arm} gripper: MOVE IT BY HAND through its FULL open<->close")
    print(f">>> travel repeatedly for the next {seconds:.0f} seconds ...\n")

    v = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
    mn = mx = v
    t_end = time.time() + seconds
    while time.time() < t_end:
        v = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
        mn, mx = min(mn, v), max(mx, v)
        print(f"\r  pos={v:5d}   min={mn:5d}   max={mx:5d}   span={mx - mn:5d}", end="", flush=True)
        time.sleep(0.02)
    print()
    bus.port_handler.closePort()

    span = mx - mn
    if span < MIN_HEALTHY_SPAN:
        print(f"\nWARNING: span is only {span} ticks (< {MIN_HEALTHY_SPAN}).")
        print("The trigger probably wasn't moved far enough. NOT saving. Re-run and")
        print("squeeze the gripper fully open and fully closed a few times.")
        return 1

    with open(calib) as f:
        cal = json.load(f)
    old = cal["gripper"].copy()
    cal["gripper"]["range_min"] = int(mn)
    cal["gripper"]["range_max"] = int(mx)
    with open(calib, "w") as f:
        json.dump(cal, f, indent=4)
        f.write("\n")

    print(f"\nSaved {calib}")
    print(f"  gripper range_min: {old['range_min']} -> {int(mn)}")
    print(f"  gripper range_max: {old['range_max']} -> {int(mx)}   (span {span})")
    print("Homing offset and the five arm joints were left unchanged.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
