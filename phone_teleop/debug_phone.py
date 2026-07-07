#!/usr/bin/env python
"""Print the raw phone/WebXR state each frame — isolates the input from the robot.

    uv run python phone_teleop/debug_phone.py

Open the printed URL in the Quest browser, tap Start, then hold "Move" and move
around. Watch for:
  - enabled  flips True while you hold Move  (else the arm never moves)
  - pos      changes as you move             (else no 6-DOF pose is streaming)
  - inputs   reservedButtonA/B change on A/B  (the gripper controls)

Report which of those move and which stay put.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # phone_teleop/ on path
import _hebi_stub  # noqa: F401,E402

import numpy as np  # noqa: E402

from lerobot.teleoperators.phone import Phone, PhoneConfig  # noqa: E402
from lerobot.teleoperators.phone.config_phone import PhoneOS  # noqa: E402


def main():
    phone = Phone(PhoneConfig(phone_os=PhoneOS.ANDROID))
    phone.connect()
    print("\nConnected. On the Quest: tap Start, then HOLD Move and move around.\n"
          "(Ctrl-C to stop.)\n")
    last = None
    try:
        while True:
            a = phone.get_action()
            if not a:
                line = "get_action() -> {}  (no pose yet — engage Move / move the device)"
            else:
                en = a.get("phone.enabled")
                pos = a.get("phone.pos")
                rot = a.get("phone.rot")
                inp = a.get("phone.raw_inputs")
                posr = np.round(np.asarray(pos), 3) if pos is not None else None
                rv = np.round(rot.as_rotvec(), 2) if rot is not None and hasattr(rot, "as_rotvec") else rot
                line = f"enabled={en}  pos={posr}  rotvec={rv}  inputs={inp}"
            if line != last:  # only print on change, to cut spam
                print(line)
                last = line
            time.sleep(0.05)
    finally:
        phone.disconnect()


if __name__ == "__main__":
    main()
