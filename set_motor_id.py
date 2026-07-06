#!/usr/bin/env python
"""Assign the correct Feetech ID to a SINGLE SO-101 motor (follower OR leader).

Connect the controller board to exactly ONE motor, then run:

    uv run python set_motor_id.py <joint> [port]

where <joint> is one of:
    shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper

and [port] is optional (default: the follower port from so101_config). Pass it
explicitly when both arms are connected so you don't write an ID to the wrong
arm -- e.g.

    uv run python set_motor_id.py gripper "$SO101_LEADER_PORT"

The SO-101 follower and leader share the same joint->ID map (1..6) and the same
sts3215 motors, so this one script IDs either arm.

It writes that joint's canonical ID (and the default 1 Mbps baud) to whatever
single motor is currently on the bus, then verifies by pinging the new ID.

This is the same operation as `lerobot-setup-motors`, done one motor at a time.
"""

import sys

from lerobot.motors.feetech import FeetechMotorsBus

from so101_config import FOLLOWER_PORT, SO101_MOTORS as JOINTS


def main() -> int:
    if len(sys.argv) not in (2, 3) or sys.argv[1] not in JOINTS:
        print(__doc__)
        print(f"valid joints: {', '.join(JOINTS)}")
        return 2

    joint = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) == 3 else FOLLOWER_PORT
    target_id = JOINTS[joint].id
    print(f"Using port {port}")
    bus = FeetechMotorsBus(port, JOINTS)

    # Safety: make sure exactly ONE motor is on the bus before writing an ID.
    bus._connect(handshake=False)
    bus.set_baudrate(1000000)
    present = [mid for mid in range(0, 16) if bus.ping(mid, num_retry=2) is not None]
    if len(present) != 1:
        print(f"ABORT: expected exactly 1 motor on the bus, found IDs {present}.")
        print("Connect only the one motor you're assigning, then retry.")
        bus.port_handler.closePort()
        return 1
    print(f"Found 1 motor at ID {present[0]}. Assigning '{joint}' -> ID {target_id} ...")

    # setup_motor scans for the lone motor, disables torque, writes ID + default baud.
    bus.setup_motor(joint)

    # Verify
    model = bus.ping(target_id, num_retry=2)
    if model is not None:
        print(f"OK: '{joint}' is now ID {target_id} (model {model}) @ 1 Mbps.")
        return 0
    print(f"WARNING: could not confirm ID {target_id} after write. Re-scan to check.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
