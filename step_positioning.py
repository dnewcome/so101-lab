#!/usr/bin/env python
"""Read positioning targets out of a STEP file.

    uv run python step_positioning.py <part.step>

Extracts geometry useful for driving an arm to features on a part: the overall
bounding box + center, and the centers/axes of cylindrical faces (holes and
bosses). This is the "where are the features" half of the problem.

The other half -- turning a CAD-frame point into a robot command -- needs a
transform from the part's coordinate frame into the robot's base frame (a
workspace / hand-eye calibration). That lives in `cad_to_robot()` below as a
clearly-marked placeholder; fill in the 4x4 once you've registered the part on
the table (e.g. touch the gripper to 3+ known points and solve for the rigid
transform).

STEP parsing uses build123d (OpenCascade). Install the optional extra:

    uv sync --extra step        # or: uv pip install build123d
"""

import sys
from dataclasses import dataclass


@dataclass
class Hole:
    center: tuple[float, float, float]  # mm, CAD frame
    axis: tuple[float, float, float]    # unit direction of the cylinder axis
    radius: float                       # mm


def load_step(path: str):
    """Load a STEP file into a build123d shape (lazy import so the module
    imports without build123d present)."""
    from build123d import import_step

    return import_step(path)


def bounding_box(shape) -> dict:
    bb = shape.bounding_box()
    return {
        "min": tuple(bb.min.to_tuple()),
        "max": tuple(bb.max.to_tuple()),
        "center": tuple(bb.center().to_tuple()),
        "size": tuple(bb.size.to_tuple()),
    }


def holes(shape, max_radius: float | None = None) -> list[Hole]:
    """Cylindrical faces (holes / round bosses), reported by center + axis.

    Pass `max_radius` to keep only holes at or below a size (e.g. filter out
    large bores or the part's outer round wall).
    """
    from build123d import GeomType

    found: list[Hole] = []
    for face in shape.faces():
        if face.geom_type != GeomType.CYLINDER:
            continue
        radius = getattr(face, "radius", None)
        if radius is None:
            continue
        if max_radius is not None and radius > max_radius:
            continue
        c = face.center()
        n = face.normal_at(c)  # axis direction proxy
        found.append(
            Hole(center=tuple(c.to_tuple()), axis=tuple(n.to_tuple()), radius=float(radius))
        )
    return found


def cad_to_robot(point_mm):
    """Map a CAD-frame point (mm) to the robot base frame.

    PLACEHOLDER -- returns the point unchanged. Replace with your registered
    4x4 rigid transform (rotation + translation) once the part's placement on
    the table is known. Keep units consistent with whatever your motion code
    expects (lerobot SO-101 joint targets are in degrees; a Cartesian target
    needs your own IK).
    """
    return point_mm


def summarize(path: str) -> None:
    shape = load_step(path)
    bb = bounding_box(shape)
    print(f"STEP: {path}")
    print(f"  bbox min={_fmt(bb['min'])}  max={_fmt(bb['max'])}")
    print(f"  center={_fmt(bb['center'])}  size={_fmt(bb['size'])} mm")
    hs = holes(shape)
    print(f"  cylindrical faces (holes/bosses): {len(hs)}")
    for i, h in enumerate(sorted(hs, key=lambda x: x.radius)):
        print(f"    [{i}] r={h.radius:6.2f}  center={_fmt(h.center)}  axis={_fmt(h.axis)}")


def _fmt(t) -> str:
    return "(" + ", ".join(f"{v:7.2f}" for v in t) + ")"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    try:
        summarize(sys.argv[1])
    except ImportError:
        print("build123d not installed. Run:  uv sync --extra step")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
