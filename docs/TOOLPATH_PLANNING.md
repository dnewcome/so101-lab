# Toolpath tracing & interference-free motion — options

Notes for driving the SO-101 (and eventually multiple arms) along CAD-derived
paths, and what open-source tooling exists for collision-free motion on 6-DOF
arms. Written while building the first slice (`trace_path.py`).

## TL;DR

- **lerobot can *execute* a toolpath but not *plan* one.** It ships placo-based
  FK/IK (`lerobot.model.kinematics.RobotKinematics`) and an end-effector
  control pipeline for the SO-101, but **no collision checking and no motion
  planner**. It's the execution/learning layer.
- **The SO-101 is a 5-DOF arm** (+ gripper). It can't hold an arbitrary tool
  orientation across a workspace — it's a good match for **position-primary**
  process paths (pen, glue/paste nozzle, holding a part) where the tool may
  tilt/spin, not for full 6-DOF pose tracking or anything needing stiffness.
- **First slice built:** `trace_path.py` = waypoints → iterated placo IK →
  joints → follower. Dry-run tracks a demo path to **~0.06 mm** position.
- **Next layers (pick per need):** collision-aware planning via **cuRobo**
  (GPU, fits the 4070), **Tesseract/Descartes** (process paths), or **MoveIt 2**
  / **Drake**; author paths from CAD via build123d (`step_positioning.py`).

## What lerobot actually provides (grounded in this tree)

| Piece | File | Notes |
|---|---|---|
| FK + IK (placo) | `model/kinematics.py` `RobotKinematics` | URDF-driven. IK is a **single differential step** — iterate it (we do ~120x) to converge. `--extra placo-dep`. |
| EE control pipeline | `robots/so_follower/robot_kinematic_processor.py` | `EEReferenceAndDelta`, `EEBoundsAndSafety`, `inverse_kinematics_ee_to_joints`. Powers the `keyboard_ee` teleop. |
| Canonical URDF | (external) | `TheRobotStudio/SO-ARM100 → Simulation/SO101/so101_new_calib.urdf`, EE frame `gripper_frame_link`. `./fetch_urdf.sh` grabs it. placo needs the meshes, not just the XML. |
| **Missing** | — | No collision model, no self/scene collision checks, no path planner, no CAM. By design. |

## The open-source landscape

### A. Collision-aware motion planning (general)

| Tool | What | Fit for SO-101 |
|---|---|---|
| **MoveIt 2** (ROS 2) | The standard: IK (TRAC-IK/bio_ik), FCL collision vs URDF meshes + planning-scene, OMPL planners, `computeCartesianPath` for waypoint toolpaths | Most complete; heaviest (ROS 2). |
| **cuRobo** (NVIDIA, BSD) | **GPU** collision-free motion gen + IK + MPC | Strong fit — you have a 4070. Fast, Python. **Top pick for collision-free.** |
| **Drake** (TRI) | Global IK, trajectory optimization, GCS certifiably collision-free planning | Research-grade, no ROS, Python/C++. |
| **OMPL** / **Pinocchio** | Planner library / fast kinematics-dynamics | Building blocks under the above. |

### B. Process / toolpath-oriented ("robot CAM")

| Tool | What | Fit |
|---|---|---|
| **Tesseract + Descartes** (ROS-Industrial) | *The* open stack for Cartesian **process paths**: Descartes finds collision-free joint trajectories along waypoints **exploiting pose freedom** (tool spin / angular tolerance). Built for weld/sand/paint/additive. | Best open match for "trace a CAD toolpath on an arm." |
| **compas_fab** (ETH COMPAS) | **Python-first robotic fabrication**, CAD/design-driven (Rhino/Grasshopper), wraps MoveIt + analytic IK | Friendliest for CAD→arm paths. |
| **RoboDK** | Industry benchmark: offline programming + robot machining/CAM from CAD, collision avoidance, Python API | **Proprietary** (free tier). Reference point. |
| **FreeCAD Path/CAM** | CNC G-code toolpaths from CAD (3-/light-5-axis) | Authors path *geometry*; not arm kinematics — post-process into arm motion. |

### C. Simulate / validate before touching hardware

**CoppeliaSim** (free edu; built-in OMPL path planning + GUI) · **PyBullet** ·
**MuJoCo** · **NVIDIA Isaac Lab** · **Gazebo** · **Drake**.

### D. Teleop input devices (data collection / direct drive)

Orthogonal to planning — how a *human* drives the arm(s):

| Input | What | Notes |
|---|---|---|
| **SO-101 leader** (have it) | Kinematic twin; joint-space copy | Already working here. |
| **Oculus Quest 2 + [beavr-bot](https://github.com/ARCLab-MIT/beavr-bot)** (ARCLab-MIT) | VR hand/controller pose → robot EE via retargeting + IK | Gives a **6-DOF EE target** in the air; needs an **IK layer to map pose→joints — the same `RobotKinematics` we just wired**. Good for intuitive teleop + multi-arm demos; integration is its own sub-project (networking + pose retargeting + the 5-DOF reachability limits above still apply). |
| **Gamepad / keyboard_ee** | lerobot built-in EE teleop | Cheapest Cartesian jog. |

## Recommended pipeline for this stack

```
CAD model
   │  step_positioning.py (build123d): bbox, hole/feature targets  ← authoring
   ▼
Cartesian waypoints (part frame)
   │  cad_to_robot(): register part on the table (rigid 4x4)       ← calibration
   ▼
EE pose waypoints (robot base frame)
   │  PLAN:
   │   • now  → placo IK per waypoint (trace_path.py) — NO collision
   │   • next → cuRobo / Tesseract-Descartes / MoveIt — collision-free
   ▼
Joint trajectory
   │  trace_path.py --execute  (or lerobot SOFollower.send_action)
   ▼
SO-101 follower moves
```

## Status & next steps

- **Done:** `trace_path.py` — waypoints → iterated placo IK → follower.
  Dry-run: demo square tracks to ~0.06 mm (position-primary). `--execute`
  streams to the arm.
- **Next, in rough order:**
  1. `step_positioning.py` → real waypoints (feed CAD feature centers into
     `trace_path` instead of the demo shapes).
  2. Solve the `cad_to_robot()` table registration (touch 3+ known points).
  3. Add a **collision layer** — evaluate **cuRobo** first (GPU), else
     Tesseract/Descartes for true process paths.
  4. Multi-arm: extend `multiarm.py`; a shared frame between arms is the
     prerequisite for handoffs.
