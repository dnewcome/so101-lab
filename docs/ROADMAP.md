# Roadmap / future work

Status of the moving pieces and what's next. Checked = working; unchecked = to do.

> **Approach note:** we train by **imitation** (ACT/diffusion), not RL, and sim
> is **MuJoCo** (not Isaac/Gazebo). The reasoning + when-to-revisit triggers are
> in [`LEARNING_AND_SIM.md`](LEARNING_AND_SIM.md).

## Done / working

- [x] **Bring-up + calibration** of both SO-101 arms (motor IDs, per-joint range
      re-mapping tools, gripper fixes). See [`SO101_BRINGUP.md`](SO101_BRINGUP.md).
- [x] **Teleoperate + record + replay** with the leader arm (joint-space).
- [x] **Toolpath tracing** (`trace_path.py`): waypoints → placo IK → follower,
      dry-run to ~0.06 mm. No collision checking yet.
- [x] **Bimanual sim** (`sim_backend.py`) + **mock VR clutch** (`vr_teleop.py`)
      driving two MuJoCo arms.
- [x] **Phone/WebXR EE teleop in sim** (`phone_teleop/teleoperate_sim.py`):
      the real lerobot phone→EE→IK pipeline driving a MuJoCo SimRobot, arm
      rendered into Rerun.
- [x] **Quest 2 works via WebXR** (no APK): `immersive-ar` passthrough session,
      right-controller 6-DOF. Confirmed mapping — **grip = Move (clutch)**,
      **A/B = gripper**, thumbstick = scale. Wear the headset for stable tracking.

## Teleop & data collection

- [ ] **Phone/WebXR on HARDWARE**: run `phone_teleop/teleoperate.py` on the real
      follower, worn (immersive-AR passthrough so you see the arm).
- [ ] **Quest axis remap**: lerobot's `MapPhoneActionToRobotAction` sign-flips are
      tuned for a phone held flat; add a Quest-controller-frame remap so
      right→right / up→up feels natural. Tune `end_effector_step_sizes`.
- [ ] **Full learning loop on real data**: `record.py` → `lerobot-train` (ACT)
      → `evaluate.py`.
- [ ] **Wayland re-record**: lerobot's record keys (→/←/ESC) use pynput, which
      Wayland blocks (episodes only advance on the timer). Add a stdin/other
      input path so bad episodes can be redone.

## Bimanual (the two-hand goal)

- [ ] **`oculus_reader` path**: native Quest 6-DOF for *both* controllers,
      feeding `vr_teleop.py` `OculusSource`. More stable than WebXR and gives two
      independent hands.
  - [x] **Tuning + instrumentation in place** for judging the feel: `--axis-map`
        (Quest→base frame remap), `--pos-scale` (delta gain), and `--rerun`
        (per-hand latency `compute_ms`, `track_err_mm`, commanded-vs-achieved EE).
        See [`VR_TELEOP.md`](VR_TELEOP.md) "Tuning & measuring the feel". Next:
        put the headset on and tune the axis map + scale against the Rerun plots.
- [ ] **Second follower arm** (or repurpose the leader) for real bimanual.
- [ ] **Bimanual record → train** (`bi_so_follower`, or a direct `LeRobotDataset`
      writer from the `vr_teleop` loop).
- [ ] **Hold-and-glue task**: one arm holds a piece while another applies hot
      glue — needs genuine two-arm temporal overlap.

## Toolpaths & CAD

- [ ] **`step_positioning.py` → `trace_path.py`**: feed CAD hole/feature centers
      in as waypoints.
- [ ] **`cad_to_robot()` registration**: solve the CAD-frame → robot-base rigid
      transform (touch 3+ known points).
- [ ] **Collision-aware planning**: evaluate **cuRobo** (GPU, fits the 4070)
      first, else **Tesseract/Descartes** for process paths. See
      [`TOOLPATH_PLANNING.md`](TOOLPATH_PLANNING.md).

## Vision & housekeeping

- [ ] **Wrist camera** (2nd cam) for occlusion-robust grasping; re-record if adopted.
- [ ] Push recorded datasets to the **HF Hub**.
- [ ] Retire the duplicate script copies still living in the `lerobot-arm` clone;
      run everything from this repo.
