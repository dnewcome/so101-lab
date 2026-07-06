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
./fetch_urdf.sh                     # SO-101 URDF + MJCF + meshes -> ./urdf/
uv sync --extra kin --extra sim     # placo + mujoco

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
