"""Shared SO-101 kinematics: load placo FK/IK and iterate it to convergence.

Used by both `trace_path.py` (toolpaths) and `vr_teleop.py` (VR teleop) so the
sim and hardware paths share one IK. `lerobot.model.kinematics.RobotKinematics`
runs a single differential IK step per call, so we iterate it here.

    ./fetch_urdf.sh          # SO-101 URDF + meshes
    uv sync --extra kin      # placo
"""

import numpy as np

from lerobot.model.kinematics import RobotKinematics

from so101_config import URDF_PATH, URDF_TARGET_FRAME


def load_kin(urdf: str = URDF_PATH, frame: str = URDF_TARGET_FRAME) -> RobotKinematics:
    return RobotKinematics(urdf, target_frame_name=frame)


def solve_ik(kin, q_seed, T, iters: int = 120, tol_mm: float = 0.3, orientation_weight: float = 0.0):
    """Iterate the differential IK toward EE pose T (4x4). Returns (q_deg, pos_err_mm).

    orientation_weight defaults to 0 (position-only): on a 5-DOF arm you usually
    can't hold full tool orientation across a workspace, so position wins and the
    tool tilts. Raise it to trade reach for orientation.
    """
    q = np.array(q_seed, dtype=float)
    err = float("inf")
    for _ in range(iters):
        q = kin.inverse_kinematics(q, T, position_weight=1.0, orientation_weight=orientation_weight)
        err = np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - T[:3, 3]) * 1000.0
        if err < tol_mm:
            break
    return q, err
