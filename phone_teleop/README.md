# Phone / WebXR teleop → record → replay → train (single arm)

Drive the SO-101 follower's **end-effector** with a phone **or the Quest 2
browser** (both via WebXR), and get the full lerobot learning loop: teleop →
record a `LeRobotDataset` → replay → `lerobot-train` → evaluate. Single arm
(one pose → one EE); bimanual is a separate build.

Adapted from lerobot's `examples/phone_to_so100/`, wired to this repo:
SO101Follower on the `so101_config` port/id, `PhoneOS.ANDROID` (WebXR), the
fetched URDF, MJPG camera, local dataset by default.

## Setup

```bash
./setup.sh          # installs the `phone` (WebXR) extra + placo + URDF
```

## 0. Try it in SIM first (no hardware follower)

Drive a **MuJoCo** SO-101 with the phone/Quest — same lerobot pipeline, no
serial arm. Great for dialing in the WebXR loop safely.

```bash
uv run python phone_teleop/teleoperate_sim.py       # opens a MuJoCo window
uv run python phone_teleop/sim_robot.py --view      # sanity-check the sim arm (no phone)
```

`sim_robot.py` is a tiny MuJoCo "robot" that quacks like `SOFollower`
(`get_observation`/`send_action`), so `teleoperate_sim.py` runs the *exact* EE
pipeline — verified: a controller offset moves the sim EE 1:1. Once it feels
right in sim, use the hardware script below.

## 1. Teleoperate on hardware (dial it in)

```bash
uv run python phone_teleop/teleoperate.py
```

It prints a **local URL**. On the **Quest 2** (or an Android phone) — same
Wi-Fi as the PC — open that URL in the **browser**, tap **Start**, then
**press-and-hold "Move"** to drive the arm. The first press **latches the
reference pose** (that's the clutch); release to freeze, reposition, re-grip.
Buttons **A/B** open/close the gripper.

- **Orientation:** hold the controller/phone with its top edge pointing the
  same way as the gripper, so motion feels aligned.
- **Quest, no headset on your head:** put it on a stand facing the bench,
  defeat the proximity sensor so it stays awake, keep the controller in the
  cameras' view (see the main README's VR notes).
- Tune feel in `teleoperate.py`: `end_effector_step_sizes` (speed),
  `EEBoundsAndSafety` (workspace + max step).

## 2. Record

Edit the CONFIG block in `record.py` (task, episodes, repo id — local by
default), then:

```bash
uv run python phone_teleop/record.py
```

Saves absolute **EE observations + actions** to
`~/.cache/huggingface/lerobot/<HF_REPO_ID>`. Vary the object position between
episodes.

- **Wayland:** the next/redo/stop keys (pynput) don't register — episodes
  advance on the `EPISODE_TIME_SEC` timer (same as before). Ctrl-C to stop.
- Re-recording the same `HF_REPO_ID` needs the old dataset dir removed first.

## 3. Replay (no phone/headset needed)

```bash
uv run python phone_teleop/replay.py     # set HF_REPO_ID / EPISODE_IDX to match
```

Streams the recorded EE actions back through open-loop IK.

## 4. Train + evaluate

```bash
uv run lerobot-train \
  --dataset.repo_id=dan/so101_phone_test \
  --policy.type=act \
  --output_dir=outputs/train/so101_phone_act \
  --job_name=so101_phone_act \
  --policy.device=cuda
```

Then evaluate the checkpoint back on the arm (see lerobot's
`examples/phone_to_so100/evaluate.py`, or `lerobot-eval`), which runs the same
EE→IK pipeline with the trained policy in the loop.

## Troubleshooting (WebXR)

- **URL not reachable on the Quest:** use the exact IP the script prints, and
  `https` (accept the self-signed cert in the browser).
- **Motion inverted / axes swapped:** flip signs in lerobot's
  `MapPhoneActionToRobotAction`, or adjust `end_effector_step_sizes`.
- **Not discovered / no pose:** confirm PC and Quest are on the same network.
- **`hebi-py` / "HEBI Core library not found":** lerobot's phone teleop guards on
  `hebi-py` (only the iOS path uses it). We install it and stub the native module
  in `_hebi_stub.py` (imported first in `teleoperate.py`/`record.py`). If you
  write your own phone script, `import _hebi_stub` before importing
  `lerobot.teleoperators.phone`.
