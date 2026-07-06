#!/usr/bin/env python
"""Map the FOLLOWER gripper's true range by driving it to its mechanical stops.

    uv run python find_follower_gripper.py

The follower gripper won't back-drive by hand (too much gear friction), so
`calibrate_gripper.py` (which relies on hand movement) can't map it. Instead
this drives the servo in position mode, ramping toward each stop until
`Present_Position` can no longer follow the goal (lag > threshold = mechanical
limit), records the two stop positions, insets a tiny margin, and patches the
follower calibration's `gripper.range_min/range_max`.

Why the follower gripper needs this at all: its original `lerobot-calibrate`
range was under-swept (e.g. 1810-2048, ~238 ticks) so it wouldn't close all the
way in teleop. The true travel is ~680 ticks.

FRAME NOTE: it also copies the gripper's *current hardware* `Homing_Offset`
into the JSON, so the saved range and the homing offset live in the same frame.
Skipping that caused a subtle bug where teleop rewrote the homing offset on
connect and shifted the range out from under itself.

WARNING: the gripper opens/closes on its own while this runs -- keep fingers clear.
"""
import json
import os
import time

from lerobot.motors.feetech import FeetechMotorsBus

from so101_config import FOLLOWER_CALIB as CAL, FOLLOWER_PORT as PORT, SO101_MOTORS

JOINTS = {"gripper": SO101_MOTORS["gripper"]}

STEP = 4          # ticks per command
DWELL = 0.03      # s between commands
ERR_THRESH = 15   # ticks of lag that counts as "not following"
STALL_HITS = 3    # consecutive lagging reads => hit the stop
MAX_TRAVEL = 700  # ticks to search in each direction
INSET = 2         # back off this many ticks from each hard stop (near-full close)


def drive_to_stop(bus, direction, start):
    goal = start
    for _ in range(MAX_TRAVEL // STEP):
        goal = max(20, min(4075, goal + direction * STEP))
        bus.write("Goal_Position", "gripper", goal, normalize=False)
        time.sleep(DWELL)
        present = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
        if abs(goal - present) > ERR_THRESH:
            hits = 1
            for _ in range(STALL_HITS - 1):
                bus.write("Goal_Position", "gripper", goal, normalize=False)
                time.sleep(DWELL)
                present = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
                if abs(goal - present) > ERR_THRESH:
                    hits += 1
            if hits >= STALL_HITS:
                break
    stop = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
    bus.write("Goal_Position", "gripper", stop, normalize=False)  # stop pushing
    return stop


def main() -> int:
    bus = FeetechMotorsBus(PORT, JOINTS)
    bus._connect(handshake=False)
    bus.set_baudrate(1000000)

    hw_home = bus.read("Homing_Offset", "gripper", normalize=False, num_retry=2)
    orig_min = bus.read("Min_Position_Limit", "gripper", normalize=False, num_retry=2)
    orig_max = bus.read("Max_Position_Limit", "gripper", normalize=False, num_retry=2)
    print(f"hardware: homing={hw_home}  limits=[{orig_min}, {orig_max}]")

    # Open the angle limits so we can drive to the true mechanical stops.
    bus.write("Min_Position_Limit", "gripper", 0, normalize=False)
    bus.write("Max_Position_Limit", "gripper", 4095, normalize=False)

    bus.write("Operating_Mode", "gripper", 0, normalize=False)  # position
    start = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
    bus.write("Goal_Position", "gripper", start, normalize=False)
    bus.write("Torque_Enable", "gripper", 1, normalize=False)
    time.sleep(0.2)

    print(f"start={start}; driving toward CLOSE...")
    hi = drive_to_stop(bus, +1, start)
    print(f"  close stop ~ {hi}")
    time.sleep(0.2)
    mid = bus.read("Present_Position", "gripper", normalize=False, num_retry=2)
    print("driving toward OPEN...")
    lo = drive_to_stop(bus, -1, mid)
    print(f"  open stop ~ {lo}")

    # Park mid-range, then relax torque.
    bus.write("Goal_Position", "gripper", (hi + lo) // 2, normalize=False)
    time.sleep(0.3)
    bus.write("Torque_Enable", "gripper", 0, normalize=False)

    new_min = min(lo, hi) + INSET
    new_max = max(lo, hi) - INSET
    span = new_max - new_min
    # Restore hardware limits to the new usable range.
    bus.write("Min_Position_Limit", "gripper", new_min, normalize=False)
    bus.write("Max_Position_Limit", "gripper", new_max, normalize=False)
    bus.port_handler.closePort()

    print(f"\nmeasured stops [{min(lo, hi)}, {max(lo, hi)}] -> range [{new_min}, {new_max}] span={span}")
    if span < 150:
        print(f"WARNING: span {span} < 150; not saving (gripper may not have moved).")
        return 1

    with open(CAL) as f:
        cal = json.load(f)
    old = cal["gripper"].copy()
    cal["gripper"]["homing_offset"] = int(hw_home)  # keep range + homing in one frame
    cal["gripper"]["range_min"] = int(new_min)
    cal["gripper"]["range_max"] = int(new_max)
    with open(CAL, "w") as f:
        json.dump(cal, f, indent=4)
        f.write("\n")
    print(f"\nSaved {CAL}")
    print(f"  range_min: {old['range_min']} -> {int(new_min)}")
    print(f"  range_max: {old['range_max']} -> {int(new_max)}   (span {span})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
