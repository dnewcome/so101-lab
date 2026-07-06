#!/usr/bin/env bash
# One-shot setup for so101-lab.
#
#   ./setup.sh              # base + sim (mujoco) + placo + URDF
#   ./setup.sh --step       # also install build123d (STEP-file reading)
#
# Why placo is installed separately: uv's *locked* resolver can't handle placo
# 0.9.23's cmeel pre-release dependency stack (and 0.9.16 fails to import for a
# missing system liburdfdom). `uv pip install` resolves it fine, so we do that
# after `uv sync`. Re-run this script if a later `uv sync` ever prunes placo.
set -euo pipefail
cd "$(dirname "$0")"

EXTRAS=(--extra sim)
[[ "${1:-}" == "--step" ]] && EXTRAS+=(--extra step)

echo "==> uv sync ${EXTRAS[*]}  (builds lerobot from git; slow first time)"
uv sync "${EXTRAS[@]}"

echo "==> uv pip install placo==0.9.23  (kinematics; handles cmeel pre-releases)"
uv pip install "placo==0.9.23"

echo "==> fetch SO-101 URDF + MJCF + meshes"
./fetch_urdf.sh

echo
echo "Setup complete. Try:"
echo "  uv run python vr_teleop.py --view      # two arms in MuJoCo, mock controllers"
echo "  uv run python trace_path.py            # dry-run a toolpath"
