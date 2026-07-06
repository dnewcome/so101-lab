#!/usr/bin/env python
"""Keyboard joint-jog for the SO-101 follower (no leader arm needed).

Drives each joint directly from terminal key presses. Reads keys via raw
stdin (termios) instead of pynput/X11, so it works on Wayland -- just keep
this terminal window focused while you drive.

    uv run python keyboard_teleop.py                 # jog only
    uv run python keyboard_teleop.py --rerun         # + live Rerun data viewer
    uv run python keyboard_teleop.py --rerun --cam 0 # + camera feed in the viewer

With --rerun, a Rerun window opens and logs every joint angle as a live
time-series (plus the camera feed if --cam is given) while you drive.

Controls (tap a key to nudge; hold for key-repeat):

    joint            +        -
    shoulder_pan     d        a
    shoulder_lift    w        s
    elbow_flex       e        c
    wrist_flex       r        v
    wrist_roll       t        b
    gripper (open/close) g    h

    SPACE   re-hold current measured position (kill any drift)
    - / =   smaller / larger step size
    ?       reprint this help
    q / ESC quit (releases torque)

Safety: small steps + a per-command relative cap. Keep a hand near the
power switch the first time, and move gently near joint limits.
"""

import logging
import sys
import termios
import time
import tty

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

from so101_config import FOLLOWER_ID, FOLLOWER_PORT

# key -> (joint, direction). Arm joints in degrees; gripper in percent.
BINDINGS = {
    "d": ("shoulder_pan", +1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "c": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "v": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "b": ("wrist_roll", -1),
    "g": ("gripper", +1), "h": ("gripper", -1),
}
GRIPPER_LIMITS = (0.0, 100.0)


def main() -> int:
    logging.getLogger().setLevel(logging.ERROR)  # hush the per-frame clamp warnings

    # --- optional flags ---
    use_rerun = "--rerun" in sys.argv
    cam_index = None
    if "--cam" in sys.argv:
        cam_index = int(sys.argv[sys.argv.index("--cam") + 1])

    cameras = {}
    if cam_index is not None:
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        cameras = {"front": OpenCVCameraConfig(index_or_path=cam_index, fps=30, width=640, height=480)}

    log_data = None
    if use_rerun:
        from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
        init_rerun(session_name="so101_keyboard_teleop")
        log_data = log_rerun_data

    step = 2.0  # degrees (and % for gripper) per key tap
    cfg = SOFollowerRobotConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID, max_relative_target=20.0, cameras=cameras)
    robot = SOFollower(cfg)
    robot.connect(calibrate=False)
    robot.bus.enable_torque(num_retry=5)  # last-in-chain motors can glitch once

    def safe_send(action, tries=3):
        """send_action that tolerates the occasional garbled status packet."""
        for i in range(tries):
            try:
                robot.send_action(action)
                return True
            except ConnectionError:
                if i == tries - 1:
                    sys.stdout.write("\r[bus glitch -- retry failed, ignoring frame] ")
                    sys.stdout.flush()
                    return False
        return False

    obs = robot.get_observation()
    target = {k: v for k, v in obs.items() if k.endswith(".pos")}
    safe_send(target)  # hold in place

    print(__doc__)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # char-at-a-time, but keep Ctrl-C working
        while True:
            ch = sys.stdin.read(1)
            if ch in ("q", "\x1b"):  # q or ESC
                break
            elif ch == "?":
                print(__doc__)
                continue
            elif ch in ("-", "="):
                step = max(0.5, step - 0.5) if ch == "-" else min(15.0, step + 0.5)
                print(f"\nstep = {step:.1f}")
                continue
            elif ch == " ":
                obs = robot.get_observation()
                target = {k: v for k, v in obs.items() if k.endswith(".pos")}
                safe_send(target)
                print("\n[hold]")
                continue
            elif ch in BINDINGS:
                joint, sign = BINDINGS[ch]
                key = f"{joint}.pos"
                # Re-anchor to the MEASURED position so the target can't run
                # ahead of the arm (which caused clamping + "stuck joint").
                obs = robot.get_observation()
                target = {k: v for k, v in obs.items() if k.endswith(".pos")}
                target[key] = target[key] + sign * step
                if joint == "gripper":
                    target[key] = min(max(target[key], GRIPPER_LIMITS[0]), GRIPPER_LIMITS[1])
                safe_send(target)
            else:
                continue

            time.sleep(0.02)
            now = robot.get_observation()
            if log_data is not None:
                log_data(observation=now, action=target)
            line = "  ".join(
                f"{j.split('.')[0][:4]}:{now[j]:6.1f}" for j in target if j.endswith(".pos")
            )
            sys.stdout.write("\r" + line + "   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            robot.bus.disable_torque()
            robot.disconnect()
        except Exception:
            pass
        print("\ntorque released, disconnected.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
