# Bimanual VR teleop (Quest 2) — sim first

Drive two SO-101 arms with two Oculus Quest 2 controllers instead of the
hardware leader arms. Built to develop entirely in **MuJoCo** first, then swap
to hardware with a one-line change.

## The pipeline

```
Quest controller pose ─clutch─► EE target ─placo IK─► joint targets ─► arm(s)
        (pose source)                (ik.py, same solver as trace_path)     (backend)
```

Two things are pluggable; the clutch + IK never change:

| Stage | Sim (now) | Real |
|---|---|---|
| **Pose source** | `MockSource` (scripted circles — no headset needed) | `OculusSource` (Quest via `oculus_reader`) |
| **Backend** | `SimBackend` — two arms in MuJoCo (`sim_backend.py`) | `HardwareBackend` — two `SOFollower`s (`multiarm.py`) |

**Clutch** (per hand, mirrors lerobot's `EEReferenceAndDelta`): hold the **grip**
button to engage; the controller's motion *delta* becomes an EE *delta* (so no
room-to-robot calibration is needed). Release to freeze. **Trigger** → gripper.
The SO-101 is 5-DOF, so control is **position-primary** (tool may tilt).

## Run it

```bash
./setup.sh                          # uv sync (lerobot+mujoco) + placo + URDF/MJCF

uv run python sim_backend.py            # self-test: both arms track IK (~0.04 deg)
uv run python vr_teleop.py --iters 240  # mock controllers drive both sim arms
uv run python vr_teleop.py --view       # + MuJoCo viewer window (needs a display)
```

Verified headless: mock controllers drive both sim arms through 8 cm circles.

## Bringing in the real Quest 2

1. **Sideload `oculus_reader`'s APK** onto the headset (enable Developer Mode,
   connect over USB/ADB or Wi-Fi). Repo: `rail-berkeley/oculus_reader`.
2. `pip install "oculus_reader @ git+https://github.com/rail-berkeley/oculus_reader.git"`
3. `uv run python vr_teleop.py --source oculus` — now your controllers drive the
   two **sim** arms. Confirm it feels right in MuJoCo before any hardware.

## Tuning & measuring the feel (sim, headset on)

"Does two-handed teleop work well enough?" is a *feel* question, and the mock
source can't answer it — it drives scripted circles with no human in the loop.
Put the headset on (`--source oculus --backend sim`) and use these:

```bash
uv run python vr_teleop.py --source oculus --rerun          # watch it live
uv run python vr_teleop.py --source oculus --pos-scale 0.5   # finer control
uv run python vr_teleop.py --source oculus --axis-map "x=-z,y=x,z=y"  # fix a wrong axis
```

- **`--axis-map`** rotates the Quest tracking frame into the robot base frame.
  Default `x=-z,y=-x,z=y` (Quest right/up/back → base forward/left/up) is a
  *starting guess*. If moving a hand right sends the EE up/backward, flip a sign
  — each entry maps a **base** axis to a signed **Quest** axis.
- **`--pos-scale`** is the delta gain (1.0 = 1:1). Drop below 1 for finer, less
  twitchy control; raise it for more reach per hand motion.
- **`--max-step`** is the **spike limiter** — a hard cap on how far the EE may
  move per frame (metres). The arm physically can't jump further than this no
  matter how badly a controller frame glitches; a wildly out-of-range single
  frame (> ~6× the cap) is dropped entirely, and a *sustained* relocation slews
  in at the cap instead of teleporting. Lower it (e.g. `0.01`) for calmer motion
  at the cost of a little lag. Essential when controllers are poorly tracked
  (e.g. held off to the side or near the edge of the cameras' view).
- **`--smooth`** adds low-pass damping on top: `1.0` = pure rate limit (default),
  `0.5` = extra smoothing for very jittery tracking.
- **`--rerun`** logs, per hand: commanded vs achieved EE (`cmd_ee`/`achieved_ee`
  in 3D), `track_err_mm`, `clutch`, `trigger`, plus `timing/compute_ms` and
  `loop_hz`. This turns "hard to tell" into numbers:
  - **`compute_ms`** — pose-read + IK + command latency. Teleop feel falls off a
    cliff past ~100–150 ms; IK/command alone is ~0.2 ms, so anything large is the
    `oculus_reader` read (ADB/Wi-Fi). Prefer USB/ADB over Wi-Fi if it spikes.
  - **`track_err_mm`** — how far the sim arm lags the commanded EE. Large/laggy →
    the arm can't keep up (or you're near a singularity / the 5-DOF limit).
  - **`cmd_ee` vs `achieved_ee`** — if the commanded point jumps off the axis
    your hand moved along, your `--axis-map` is wrong.
  - **`raw_step_mm`** / **`guard`** — the pre-limiter commanded step and what the
    spike limiter did with it (`0` passed, `1` rate-limited, `2` held as a
    glitch). Frequent `2`s mean the controllers are being tracked badly — the
    limiter is saving you, but improving tracking (or lowering `--max-step`) is
    the real fix.

Even headless (no headset), `--iters N` prints the per-tick latency, so you can
benchmark the compute cost separately from the headset read.

## Going to hardware

You need **two follower arms**. Options:
- A second SO-101 follower (best — full torque).
- Repurpose the leader as a second follower (same servos; lower gear ratios =
  less holding torque, fine for light tasks).

Fill `ARM_TABLE` in `multiarm.py` with `left`/`right` ports+ids, then:

```bash
uv run python vr_teleop.py --source oculus --backend hardware
```

## Notes / limits

- **5-DOF**: no arbitrary tool orientation; position-primary (see
  [`TOOLPATH_PLANNING.md`](TOOLPATH_PLANNING.md)).
- **Shared IK** (`ik.py`) is used by both `vr_teleop.py` and `trace_path.py`, so
  sim and hardware behave identically.
- **beavr-bot** (ARCLab-MIT) is the heavier batteries-included alternative for
  Quest bimanual teleop; this is the minimal path that reuses our own IK.
- VR teleop can also feed `lerobot-record` for demo collection (drop the leader).
