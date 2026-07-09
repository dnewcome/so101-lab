# Learning approach & simulator choice (decision record)

Why this repo trains policies by **imitation**, not reinforcement learning, and
why the simulator is **MuJoCo** rather than NVIDIA Isaac or Gazebo. Written down
so these questions don't get re-litigated every few weeks.

## What kind of learning is this? — Imitation, not RL

The learning pipeline is **behavioral cloning** (a.k.a. imitation learning):
you teleoperate the arm, record `(image, state, action)` frames into a
`LeRobotDataset`, and train a policy (**ACT** or **diffusion policy**) to *copy*
the demonstrated actions. It is supervised learning — the human demo is the label.
There is no reward, no exploration, no trial-and-error.

Grounded in the tree:
- `README.md` — "record a `LeRobotDataset` → `replay.py` → `lerobot-train` → evaluate"
- [`SO101_BRINGUP.md`](SO101_BRINGUP.md) — "**Train** (ACT/diffusion) → `lerobot-train`"
- [`ROADMAP.md`](ROADMAP.md) — "Full learning loop on real data: `record.py` → `lerobot-train` (ACT)"

| | **Imitation learning (what we do)** | **Reinforcement learning** |
|---|---|---|
| Signal | Human demos (recorded teleop) | A **reward** function |
| How it learns | Copy the expert, supervised | Trial-and-error, maximize reward |
| Needs | A dataset of demos | An environment + reward + many rollouts |
| Our tools | `lerobot-record` + `lerobot-train` (ACT/diffusion) | Isaac Lab / massively-parallel sim |
| In this repo? | **Yes — the whole point** | **Nowhere** — no reward anywhere in the tree |

There are actually **three** non-RL things in the repo; only the first is
"learning" at all:

1. **Imitation learning** — `lerobot-train` (ACT/diffusion). The ML part.
2. **Classical motion planning / IK** — `placo` IK (`ik.py`), `trace_path.py`,
   the cuRobo/Tesseract/MoveIt roadmap. Pure math, *no* learning.
3. **Teleop** — keyboard / phone / VR. Just control + data collection.

**When RL would enter:** only if we wanted to learn a behavior from a *reward*
instead of demos (e.g. learn a grasp by trial-and-error). That is the moment
Isaac Lab (or MuJoCo + an RL library) would be worth it — see below. Nothing on
the current [roadmap](ROADMAP.md) needs it; for a 5-DOF arm with a human in the
loop, imitation learning is the far more practical route.

## Simulator / framework choice — MuJoCo now

The SO-101 is [`lerobot`](https://github.com/huggingface/lerobot)'s **reference
arm**, and our stack (MuJoCo + `lerobot` + `placo` IK) is the native, laptop-
friendly path for it. We evaluated two heavier alternatives and declined both
*for now*, with explicit triggers for when they'd become worth it.

| | **MuJoCo (current)** | **Gazebo** | **NVIDIA Isaac Sim/Lab** |
|---|---|---|---|
| Contact-rich manipulation | Best-in-class, fast | Weaker (DART/ODE/Bullet) | Good (PhysX) |
| Ecosystem fit | Native to `lerobot`/SO-101 | Native to **ROS 2** (we run zero ROS) | NVIDIA/Omniverse |
| Photoreal / synthetic data | No | No (decent ogre2, not RTX) | **Yes** — the one real reason to switch |
| Massively-parallel RL | No | No | **Yes** (Isaac Lab) |
| Setup weight | Light | Heavy (ROS 2) | Heavy (Omniverse/USD) |

Hardware is **not** the blocker: this box has an **RTX 4070 SUPER (12 GB)**, so
Isaac Sim/Lab *would* run (12 GB is a little tight for sim + train at once, but
workable). The decision is goal-fit, not capability.

### Decision rules

- **Manipulation / imitation learning** → stay on **MuJoCo**.
- **Vision sim2real / synthetic data / RL at scale** → **Isaac Sim/Lab**.
  Isaac's genuine edge is photoreal rendering + `Replicator` synthetic data +
  domain randomization for camera-based policies, plus GPU-parallel RL in Isaac
  Lab. Adopt a *specific* piece for a *specific* goal — not the whole Omniverse
  firehose.
- **Only if we commit to ROS 2** (Nav2 / MoveIt 2 on the mobile base) →
  **Gazebo**. Its whole value is tight ROS 2 integration (SDF worlds, `gz-sim`
  plugins, the `ros_gz` bridge, standard sensor sims). Without ROS it is a
  heavier simulator that is *worse* than MuJoCo at manipulation. If the
  LeKiwi-style mobile base grows into a real navigation project, Gazebo becomes
  the natural sim for *that* part.

**Isaac's four products are not one thing** — only one is compelling here:

| Isaac piece | What it buys | Worth it? |
|---|---|---|
| **Isaac Sim + Replicator** | Photoreal rendering + synthetic/labeled data + domain randomization | **Only compelling reason** — *if* we go vision-based sim2real |
| **Isaac Lab** | Thousands of parallel envs for **RL** on GPU | Only if we switch to RL (we do imitation) |
| **Isaac ROS** (perception GEMs) | HW-accelerated VSLAM/nvblox/DNN, Jetson deploy | No — our stack is lerobot-Python, not ROS 2 |
| **Isaac GR00T** | Foundation-model manipulation policies (lerobot interop exists) | Maybe later — heavyweight, humanoid-leaning |

## TL;DR

- We do **imitation learning** (ACT/diffusion via `lerobot-train`), **not RL**.
- Simulator is **MuJoCo** and stays that way for manipulation.
- **Isaac** only if the next goal is vision sim2real / RL at scale (GPU is ready).
- **Gazebo** only if the mobile base commits to **ROS 2** (Nav2/MoveIt).
