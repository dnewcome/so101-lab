#!/usr/bin/env python
"""Coordinate several SO-101 follower arms from one process.

A thin wrapper that connects N `SOFollower` robots (each on its own serial port
and calibration id) and lets you read/command them together. This is the
building block for multi-arm tasks (e.g. one arm holds a part while another
works on it).

    uv run python multiarm.py            # connect all arms in ARM_TABLE, print state

Two ways to do bimanual on SO-101:
  1. This module -- independent SOFollower objects you drive yourself. Most
     flexible (any number of arms, mixed roles, your own coordination logic).
  2. lerobot's built-in `bi_so_follower` robot -- a single robot object that
     wraps a left+right pair, if you want the stock bimanual record/train path.

Coordination is only as good as a SHARED FRAME: to have arm A hand off to arm B,
both arms' base frames must be related by a known transform. Measure it once
(touch both grippers to the same physical points) and store it here.
"""

import time

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

from so101_config import ARMS

# name -> (port, calibration id). Extend this as you add follower arms.
# The default reuses the single follower from so101_config; add more entries
# like  "right": ("/dev/serial/by-id/...", "my_so101_right").
ARM_TABLE = {
    "main": (ARMS["follower"]["port"], ARMS["follower"]["id"]),
}


class MultiArm:
    """Connect and drive several SOFollower arms together."""

    def __init__(self, table: dict[str, tuple[str, str]] = ARM_TABLE):
        self.table = table
        self.arms: dict[str, SOFollower] = {}

    def connect(self, calibrate: bool = False) -> None:
        for name, (port, arm_id) in self.table.items():
            cfg = SOFollowerRobotConfig(port=port, id=arm_id, max_relative_target=20.0)
            robot = SOFollower(cfg)
            robot.connect(calibrate=calibrate)
            self.arms[name] = robot

    def observe(self) -> dict[str, dict]:
        """Current observation (joint positions etc.) for every arm."""
        return {name: robot.get_observation() for name, robot in self.arms.items()}

    def send(self, actions: dict[str, dict]) -> None:
        """Send a per-arm action dict: {arm_name: {"<joint>.pos": value, ...}}.

        Arms are commanded in sequence; for tight synchronization you'd thread
        these or interleave the bus writes -- start simple, measure, then
        optimize only if the lag matters for your task.
        """
        for name, action in actions.items():
            if name in self.arms:
                self.arms[name].send_action(action)

    def disconnect(self) -> None:
        for robot in self.arms.values():
            try:
                robot.bus.disable_torque()
                robot.disconnect()
            except Exception:
                pass
        self.arms.clear()


def main() -> int:
    ma = MultiArm()
    ma.connect(calibrate=False)
    try:
        print(f"connected arms: {', '.join(ma.arms)}")
        time.sleep(0.2)
        for name, obs in ma.observe().items():
            pos = {k: round(v, 1) for k, v in obs.items() if k.endswith(".pos")}
            print(f"  {name}: {pos}")
    finally:
        ma.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
