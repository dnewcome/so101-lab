#!/usr/bin/env bash
# Fetch the SO-101 URDF + meshes into ./urdf/ (gitignored).
#
# Source: TheRobotStudio/SO-ARM100  (Apache-2.0)  Simulation/SO101/
# placo/pinocchio needs the STL meshes, not just the URDF XML, so we grab the
# whole Simulation/SO101 folder via a blobless sparse checkout (small + fast).
#
#   ./fetch_urdf.sh
#   -> urdf/so101_new_calib.urdf   (+ urdf/assets/*.stl)
set -euo pipefail

DEST="${1:-urdf}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Cloning SO-ARM100 (Simulation/SO101 only) ..."
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/TheRobotStudio/SO-ARM100.git "$TMP/SO-ARM100"
git -C "$TMP/SO-ARM100" sparse-checkout set Simulation/SO101

mkdir -p "$DEST"
cp -r "$TMP/SO-ARM100/Simulation/SO101/." "$DEST/"

echo
echo "Done. URDF at: $DEST/so101_new_calib.urdf"
echo "(point SO101_URDF at it, or leave the default in so101_config.py)"
