# SO-101 Bring-Up Notes

Working notes for getting an **SO-101 follower arm** running with
[lerobot](https://github.com/huggingface/lerobot) on this machine.

> Not the upstream project README — that's `README.md`. This file is our
> hardware bring-up log.

## Environment

| Thing | Value |
|---|---|
| OS | Ubuntu 25.10 |
| Python | 3.12.9 (pyenv) |
| Package manager | `uv` 0.10.4 |
| GPU | NVIDIA RTX 4070 SUPER (12 GB) |
| lerobot | 0.5.2 (cloned from source) |
| torch | 2.11.0+cu128 — CUDA verified working |
| Arm serial ports | two CH9102 adapters (VID `1a86` / PID `55d3`), see table below |

Hardware on hand: **follower arm** (bring-up complete) + **leader arm**
(bring-up complete, on replacement board) + Logitech C920 camera. The leader
unlocks recording teleoperated demos (`lerobot-record`) — the main thing the
follower-only setup couldn't do.

## Serial port identity (two boards)

Both boards use the same CH9102 chip and the **same USB VID:PID**, so
`/dev/ttyACM0` vs `ttyACM1` is assigned by plug/enumeration order and can
**swap** across reboots. Fortunately these chips carry **unique serial
numbers**, so the `/dev/serial/by-id/` symlinks are stable — use those, not
the raw `ttyACMx`:

| Arm | Stable `by-id` name | lerobot flag |
|---|---|---|
| **Leader** (teleop) | `usb-1a86_USB_Single_Serial_5B3D049885-if00` | `--teleop.port=` |
| **Follower** (robot) | `usb-1a86_USB_Single_Serial_5B61033654-if00` | `--robot.port=` |

Identified by powering only one arm and scanning both ports (the `find-port`
method): with just the leader live, `5B3D049885` answered all six motors, so
it's the leader; the other board is the follower by elimination. No electrical
issue running both boards at once — separate USB devices, separate half-duplex
buses (the shared IDs 1–6 never collide across boards), separate power.

## Leader arm (bring-up complete ✅)

**Status:** all six motors ID'd 1–6 on the **replacement** board, full daisy
chain verified (6/6 ping `777`), and **calibrated** (`my_so101_leader`, at
`~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`). Runs off
`by-id` `5B3D049885` (see the port-identity table above). Both arms are now
ready for `lerobot-teleoperate` / `lerobot-record`.

## Teleoperation working ✅ (+ gripper calibration gotchas)

Full dual-arm teleop confirmed — the leader drives the follower through all six
joints with a smooth, full-range gripper:

```bash
uv run lerobot-teleoperate \
    --robot.type=so101_follower  --robot.id=my_so101 \
    --robot.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61033654-if00 \
    --teleop.type=so101_leader   --teleop.id=my_so101_leader \
    --teleop.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D049885-if00 \
    --fps=60
```
At each connect it asks per arm: *press ENTER to use the calibration file, or
'c' to recalibrate.* **Press ENTER** (twice, once per arm). `c` wipes the
calibration.

The **gripper** took three fixes — the five arm joints were fine from
`lerobot-calibrate`, but the gripper needed care (helper scripts below):

1. **Leader gripper jumpy** → its calibrated range was 1 tick (2046–2047): the
   spring-loaded trigger was never moved during `lerobot-calibrate`, so sensor
   noise mapped across the whole 0–100 output. Fix: re-measure while working the
   trigger (`calibrate_gripper.py leader`) → span ~1200.
2. **Follower wouldn't close all the way** → its range was under-swept
   (1810–2048, ~238 ticks) vs a true ~680-tick travel, *and* the follower
   gripper won't back-drive by hand (too geared). Fix: drive it to its
   mechanical stops with the motor (`find_follower_gripper.py`).
3. **Frame consistency** → the follower's `range_min/max` must live in the same
   frame as its `Homing_Offset`. lerobot writes the JSON homing offset to the
   servo on connect, so if you re-measure the range while the hardware homing
   differs from the JSON, teleop shifts the range out from under you. The mapper
   now copies the current hardware homing offset into the JSON alongside the
   range. Verify with: hardware `Homing_Offset` == JSON `homing_offset`.

Normalization (for reference): each motor stores `homing_offset`, `range_min`,
`range_max`. `Present_Position = Actual − Homing_Offset`, then for the gripper
(`RANGE_0_100`): `norm = (Present − range_min) / (range_max − range_min) × 100`.
A degenerate (tiny) range → huge normalized swings from tiny motion = jitter.

## Leader arm — motor placement

Motor placement per joint, recorded during assembly. Same ID scheme as the
follower (base = 1 … gripper = 6), but the **gear ratios differ per joint** —
the leader mixes STS3215 variants so proximal joints carrying more gravity
load get more torque while distal joints stay easy to back-drive by hand.

| Leader axis | Motor ID | Gear ratio |
|---|:---:|:---:|
| Base / Shoulder Pan | 1 | 1 / 191 |
| Shoulder Lift | 2 | 1 / 345 |
| Elbow Flex | 3 | 1 / 191 |
| Wrist Flex | 4 | 1 / 147 |
| Wrist Roll | 5 | 1 / 147 |
| Gripper | 6 | 1 / 147 |

Lower ratio (1/147) = more back-drivable (lighter to move by hand); higher
ratio (1/345 on shoulder_lift) = more holding torque where gravity load is
worst.

After assembly, the leader needs the **same bring-up as the follower**, on
its own serial port and its own calibration id:

1. Assign IDs 1–6 one motor at a time (`set_motor_id.py` or
   `lerobot-setup-motors`) — STS3215s all ship as ID 1 and will collide.
2. `lerobot-calibrate --teleop.type=so101_leader --teleop.port=<leader_port>
   --teleop.id=my_so101_leader` (note: leader is a **teleop**, not a robot).
3. Then `lerobot-teleoperate` / `lerobot-record` with the follower as
   `--robot.*` and the leader as `--teleop.*`.

> ✅ **RESOLVED — original leader board was defective; replaced.** The board
> that shipped with the leader kit enumerated over USB fine but its **servo
> data path was dead** (no motor responded at any baud, while the *same motor
> + cable + power* pinged fine on the follower board — a controlled A/B that
> isolated the board). A **replacement board** was fitted and works: all six
> leader motors were ID'd on it and the full chain pings 6/6. Kept below for
> the diagnostic lessons.
>
> Diagnostic dead-ends noted so they aren't repeated: a raw byte-level "echo"
> test is **not** diagnostic on these boards — the *known-good* follower board
> also returns zero bytes to a raw ping, so silence there proves nothing. Use
> the SDK ping (`FeetechMotorsBus.ping`) for ground truth, and always test
> with a *powered motor actually connected* (an empty/​unpowered bus also
> reads as "NONE").

## How it was installed

```bash
git clone --depth 1 https://github.com/huggingface/lerobot.git .
uv sync --extra feetech      # pulls feetech-servo-sdk + GPU torch (cu128)
```

Run all lerobot commands through the venv with `uv run ...` (or activate
`.venv`).

## Serial access

The user is **not** in the `dialout` group. Two fixes were applied:

```bash
sudo usermod -aG dialout $USER          # permanent, takes effect next login
sudo setfacl -m u:$USER:rw /dev/ttyACM0 # immediate, this session only
```

The ACL grant is what gives access *right now*; the group membership is the
durable fix. If a fresh login still can't open the port, re-run the
`setfacl` line (ACLs don't survive replug).

## Current status

**Bring-up COMPLETE** ✅ — all six motors ID'd (1–6), full chain verified
(5/5 pings each, model 777), calibrated (`my_so101`), and a live normalized
read through the robot stack works (`is_calibrated: True`, joint positions in
degrees). Remaining work is application-level (teleop needs a leader, vision
needs a camera, then training).

Two gotchas seen during bring-up, for future reference:
- **Motor power must be on** for calibrate/teleop — if the barrel-jack supply
  is off, the USB adapter still enumerates (`ttyACM0` appears) but every bus
  write times out with *"There is no status packet!"*. Symptom = all TIMEOUT.
- **Reseat the board→first-motor cable** if the chain goes silent after
  re-assembly; that link proved marginal.

<details><summary>Original problem (resolved): all motors at factory ID 1</summary>

**All six motors were at the factory-default ID 1**, so they collided on
the shared half-duplex bus. Evidence gathered during diagnosis:

- Broadcast scan across every baudrate → **no motors found**.
- Unicast ping to ID 1 @ 1 Mbps → model `777` (= STS3215), but only **~13%**
  of attempts succeed.
- Ping failures were **26/30 `RX_CORRUPT`, 0 timeouts** → overlapping
  responses (collision), not adapter latency or a dead bus.
- IDs 2–6 never answer (no motor has those IDs yet).

Conclusion: the arm was assembled and daisy-chained **before** each motor was
given a unique ID. Every STS3215 ships as ID 1, so they all shout at once.

</details>

## Fix: assign unique IDs (one motor at a time)

There is **no software shortcut** — a write to "ID 1" hits every colliding
motor. Each motor must be the *only* one on the bus when it gets its ID.

```bash
uv run lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
```

This is interactive. It prompts in **reverse joint order**; connect the
controller board to **only** the prompted motor, press enter, then move your
single cable to the next one:

| Prompt order | Joint | Assigned ID |
|:---:|---|:---:|
| 1 | `gripper` | 6 |
| 2 | `wrist_roll` | 5 |
| 3 | `wrist_flex` | 4 |
| 4 | `elbow_flex` | 3 |
| 5 | `shoulder_lift` | 2 |
| 6 | `shoulder_pan` | 1 |

The joint→ID mapping comes from **which physical motor is connected when
prompted**, so follow the order.

### Because the arm is already assembled

To present one motor at a time without disassembly:

1. Unplug the inter-motor daisy-chain cables so no motor feeds another.
2. Run a single 3-pin cable from the controller board to just the prompted
   motor.
3. Press enter → ID is written → move the cable to the next motor.
4. When all six are done, **re-cable the full daisy chain.**

If a step errors: zero or more-than-one motors are on the bus. Re-check that
exactly one is connected.

## Remaining bring-up sequence (after IDs are set)

```bash
# 1. Confirm all six now answer at IDs 1-6 (read-only scan)
uv run python -c "from lerobot.motors.feetech import FeetechMotorsBus; \
print(FeetechMotorsBus.scan_port('/dev/ttyACM0'))"

# 2. Calibrate (records each joint's range of motion)
uv run lerobot-calibrate --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 --robot.id=my_so101

# 3. Sanity-check motion / teleop (needs a leader for true teleop)
uv run lerobot-teleoperate --help
```

## Helper scripts in this repo

- **`set_motor_id.py <joint> [port]`** — assign one motor's Feetech ID (re-ID a
  replacement); works for follower or leader (same 1..6 map). Pass the port
  explicitly when both arms are connected.
- **`calibrate_gripper.py [leader|follower] [seconds]`** — re-measure ONLY the
  gripper range by hand and patch its `range_min/max` in the calibration JSON
  (leaves the five arm joints and homing offset alone). Good for the **leader**
  (spring trigger, squeeze it during the capture). The **follower** gripper is
  too geared to back-drive by hand — use the next script instead.
- **`find_follower_gripper.py`** — map the **follower** gripper by driving the
  servo to its mechanical stops (stall detection) and patch its range +
  hardware homing offset together (frame-consistent). Fingers clear — it moves
  on its own.
- **`calibrate_joint.py <arm> <joint> [seconds]`** — re-map ONE hand-movable
  joint's range without redoing the whole arm. Locks the other joints, frees the
  target, records its hand-swept min/max, patches that joint's range + homing
  (frame-consistent). Used to fix the follower `shoulder_pan` (base wasn't
  reaching full-left: span 1236 → 2720). Not for the follower gripper (too
  stiff — use `find_follower_gripper.py`).
- **`keyboard_teleop.py`** — drive the follower by keyboard (no leader needed).
  Reads raw stdin (Wayland-safe). Flags:
  - `--rerun` → live Rerun viewer with joint time-series + commanded action
  - `--cam 0` → also stream the webcam feed into Rerun
  - Controls: `d/a w/s e/c r/v t/b` jog joints, `g/h` gripper, SPACE hold,
    `-/=` step size, `q`/ESC quit.

## Hardware notes / state (as of last session)

- **Camera:** Logitech C920 confirmed working via OpenCV backend
  (`/dev/video0`, 640x480x3 frames land in `get_observation()`).
- **Viz:** `rerun-sdk` installed (`uv sync --extra feetech --extra viz`).
- **Gotcha — total absence of `/dev/ttyACM*`** = the arm's USB *data* cable is
  unplugged (distinct from motor-power-off, where the port still appears).
  Replug it; it returns as `/dev/ttyACM0`.
- **USB contention (SOLVED) — camera streaming corrupts the servo bus.** With
  the C920 on the same USB hub as the serial adapters, teleop/record runs a
  minute then dies with `Incorrect status packet!` on a `sync_read` (the raw
  YUYV stream at 640×480×30 ≈ 18 MB/s starves the half-duplex bus). Two fixes,
  applied together: (1) stream **compressed MJPG** — add `fourcc: MJPG` to the
  camera config (~2–3 MB/s, and the C920 negotiates it fine); (2) put the
  **camera on its own USB port/controller**, separate from the two arm adapters.
  With both, teleop is stable. Fallback: drop the camera to `fps: 15`.
- Camera dict for teleop/record:
  `--robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG}}"`

## Resume checklist (next session)

1. Plug in arm USB + motor power; confirm `ls /dev/ttyACM*` shows the port.
2. Plug in C920 (optional); `ls /dev/video*`.
3. `uv run python keyboard_teleop.py --rerun --cam 0` to drive + watch live data.
4. Next build step (not done yet): a **recorder** that writes a `LeRobotDataset`
   (images + state + action per frame) from the keyboard loop → enables
   `lerobot-dataset-viz` and `lerobot-train`.

## Later (needs more hardware)

- **Record demos** → needs a leader arm (`lerobot-record`).
- **Vision policies** → needs a USB camera (`lerobot-find-cameras`).
- **Train** (ACT/diffusion) → `lerobot-train`; the 4070 (12 GB) is plenty for
  fine-tuning on SO-101 data.

## Quick reference: rescan / re-diagnose the bus

```bash
uv run python - <<'PY'
from lerobot.motors.feetech import FeetechMotorsBus
bus = FeetechMotorsBus('/dev/ttyACM0', {})
bus._connect(handshake=False)
bus.set_baudrate(1000000)
for mid in range(1, 7):
    print(mid, bus.ping(mid, num_retry=2))
bus.port_handler.closePort()
PY
```

A healthy arm prints a model number (`777`) for each of IDs 1–6.
