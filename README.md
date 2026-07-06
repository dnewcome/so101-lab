# so101-lab

Tools and experiments for [SO-101](https://github.com/TheRobotStudio/SO-ARM100)
robot arms — motor ID assignment, calibration fixes, keyboard teleop, and
in-progress work on **multi-arm coordination** and **STEP-file-driven
positioning**.

Built **on top of [lerobot](https://github.com/huggingface/lerobot)** — lerobot
is a dependency here, not a fork. This repo is just the layer of scripts and
experiments I've built around it for my two-arm (leader + follower) SO-101 setup.

## Hardware

- **SO-101 follower** (the arm a policy drives) + **SO-101 leader** (a
  teleoperation device for collecting demos). Six Feetech STS3215 servos each.
- Two CH9102 USB-serial boards. They share a USB VID:PID, so `/dev/ttyACM*`
  numbering can swap across reboots — address the arms by their stable
  `/dev/serial/by-id/…` names instead (see `so101_config.py`).
- A USB camera (Logitech C920 here) for vision. Stream it as **MJPG** and keep
  it on its **own USB controller** — raw YUYV on a shared hub starves the servo
  bus (see the bring-up notes).

## Setup

Uses [uv](https://docs.astral.sh/uv/).

```bash
uv sync                 # installs lerobot (feetech+dataset+viz) + deps
uv sync --extra step    # also install build123d for STEP-file positioning
```

Then edit **`so101_config.py`** — set your two board `by-id` serials and your
lerobot calibration ids (or set the `SO101_LEADER_PORT` / `SO101_FOLLOWER_PORT`
/ `SO101_LEADER_ID` / `SO101_FOLLOWER_ID` env vars). Find your serials with:

```bash
ls /dev/serial/by-id/
```

## Tools

| Script | What it does |
|---|---|
| `set_motor_id.py <joint> [port]` | Assign one motor's Feetech ID (1–6), one at a time. Works for either arm. |
| `keyboard_teleop.py [--rerun] [--cam N]` | Jog the follower by keyboard (Wayland-safe raw stdin). Optional live Rerun viewer + camera. |
| `calibrate_gripper.py [leader\|follower] [secs]` | Re-measure just the gripper range by hand (good for the **leader** spring trigger). |
| `find_follower_gripper.py` | Map the **follower** gripper by driving it to its mechanical stops (it won't back-drive by hand). |
| `calibrate_joint.py <arm> <joint> [secs]` | Re-map any single hand-movable joint's range without redoing the whole arm. |

All are run with `uv run python <script.py> …`.

### Why the calibration helpers exist

`lerobot-calibrate` records each joint's range from a hand sweep — easy to
**under-sweep** a joint, which then can't reach its full travel in teleop (a
jumpy gripper, a base that won't turn all the way). These helpers re-map a
**single** joint in the correct frame (`homing_offset` + `range_min/max`
together) so you don't have to redo — and risk regressing — the whole arm. Full
war-story writeup in [`docs/SO101_BRINGUP.md`](docs/SO101_BRINGUP.md).

## Experiments (work in progress)

- **`multiarm.py`** — a `MultiArm` wrapper that connects several `SOFollower`
  arms and reads/commands them together. The building block for tasks where one
  arm holds a part while another works on it. (lerobot also ships a stock
  `bi_so_follower` bimanual robot if you want the built-in record/train path.)
- **`step_positioning.py`** — read a STEP file (via build123d) and pull out
  positioning targets: bounding box, and hole/boss centers + axes. Includes a
  `cad_to_robot()` placeholder for the CAD-frame → robot-base transform you
  calibrate once per table setup.

Both are honest starting points, not finished features.

## Record / train (via lerobot)

Recording demos and training policies use lerobot's own CLIs
(`lerobot-record`, `lerobot-train`, `lerobot-replay`). Datasets go to the
**Hugging Face Hub**, not this repo. See `docs/SO101_BRINGUP.md` for the
working record command (camera config, MJPG, stable ports).

## License

MIT — see [LICENSE](LICENSE). lerobot is Apache-2.0 and is used as a dependency.
