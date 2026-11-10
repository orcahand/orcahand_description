# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Regenerate the v2 LEFT hand as the exact sagittal (x=0) mirror of the RIGHT.

The left hand model used to be a separate Fusion360 export whose finger
abduction axes were tilted (baked into joint axis + parent body orientation +
joint origin) instead of the right hand's clean [0, 0, 1]. This script derives
every left model file from the right-hand sources so that
left == mirror(right) holds exactly:

  - v2/models/urdf/orcahand_left.urdf      from v2/models/urdf/orcahand_right.urdf
  - v2/models/mjcf/orcahand_left_body.xml  from v2/models/mjcf/orcahand_right_body.xml
  - v2/models/mjcf/orcahand_left.mjcf      from v2/models/mjcf/orcahand_right.mjcf
  - v2/models/assets/left/*.stl            from v2/models/assets/right/*.stl (a left mesh without one is removed)

Reflection across the x=0 plane with M = diag(-1, 1, 1). Because URDF rpy is
extrinsic XYZ (R = Rz(y) @ Ry(p) @ Rx(r)) and M @ Rx(r) @ M = Rx(r),
M @ Ry(p) @ M = Ry(-p), M @ Rz(y) @ M = Rz(-y), the conjugated rotation
M @ R @ M is exactly Rz(-y) @ Ry(-p) @ Rx(r). Every transform is therefore a
lexical sign flip that preserves all digits of the right-hand source:

  origin xyz  [x, y, z]     ->  [-x,  y,  z]
  origin rpy  [r, p, y]     ->  [ r, -p, -y]
  quaternion  [w, x, y, z]  ->  [ w,  x, -y, -z]
  joint axis  [ax, ay, az]  ->  [ ax, -ay, -az]   (see below)
  inertia     ixy, ixz negated; ixx/iyy/izz/iyz unchanged (URDF and MJCF)
  meshes      vertex x negated, triangle winding inverted (normals stay outward)

Joint axes use the SAME-ANGLE convention: a rotation by theta about axis a in
the right hand mirrors to a rotation by theta about -M @ a = [ax, -ay, -az] in
the mirrored frames (M @ Rot(a, theta) @ M = Rot(-M @ a, theta)). Equal joint
angles therefore produce mirror-identical poses and the joint limits carry
over unchanged, matching orca_core's v2_kinematics.yaml left section (which
stores axis = M @ a with the orca joint sign flipped -- the same rotation).

Joint/link/body names are semantic ({side}_{part} / {side}_{orca joint id}),
so left names are simply the right names with the side prefix swapped; only
the mesh file base names differ per side (MESH_NAME_MAP). The .mjcf carries
no numbers that mirror (limits are unchanged under the same-angle convention),
so the left .mjcf is a pure rename of the right one.

Usage:
    python utils/mirror_left_from_right.py            # regenerate left files
    python utils/mirror_left_from_right.py --check    # verify, write nothing

Requires numpy only (for the binary STL mirroring).
"""

import argparse
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
V2 = REPO_ROOT / "v2"

URDF_RIGHT = V2 / "models" / "urdf" / "orcahand_right.urdf"
URDF_LEFT = V2 / "models" / "urdf" / "orcahand_left.urdf"
BODY_RIGHT = V2 / "models" / "mjcf" / "orcahand_right_body.xml"
BODY_LEFT = V2 / "models" / "mjcf" / "orcahand_left_body.xml"
MJCF_RIGHT = V2 / "models" / "mjcf" / "orcahand_right.mjcf"
MJCF_LEFT = V2 / "models" / "mjcf" / "orcahand_left.mjcf"
MESH_RIGHT_DIR = V2 / "models" / "assets" / "right"
MESH_LEFT_DIR = V2 / "models" / "assets" / "left"

# Right mesh/part base names -> left equivalents (files and mjcf mesh refs).
MESH_NAME_MAP = {
    "R-Carpals": "L-Carpals",
    "I-AP-R": "I-AP-L",
    "T-TP-R": "T-TP-L",
    "R-T-AP": "L-T-AP",
}


def _neg(token: str) -> str:
    """Lexically negate one numeric token, preserving every digit."""
    if float(token) == 0.0:
        return token
    return token[1:] if token.startswith("-") else "-" + token


def _flip(vector_str: str, flip_indices: tuple) -> str:
    """Negate selected whitespace-separated components of a vector string."""
    tokens = vector_str.split()
    for i in flip_indices:
        tokens[i] = _neg(tokens[i])
    return " ".join(tokens)


def _rename(text: str) -> str:
    """Apply right->left renames (longest keys first so prefixes can't clash)."""
    for right, left in sorted(MESH_NAME_MAP.items(), key=lambda kv: -len(kv[0])):
        text = text.replace(right, left)
    return text.replace("right", "left")


def _mirror_attr(line: str, attr: str, flip_indices: tuple) -> str:
    """Mirror one vector-valued XML attribute on a single line, if present."""
    needle = f'{attr}="'
    start = line.find(needle)
    if start == -1:
        return line
    start += len(needle)
    end = line.index('"', start)
    return line[:start] + _flip(line[start:end], flip_indices) + line[end:]


# Attribute -> component indices to negate under M = diag(-1, 1, 1).
URDF_RULES = (
    ("xyz", (0,)),      # positions: x -> -x   (joint/inertial/visual origins)
    ("rpy", (1, 2)),    # rotations: pitch, yaw -> negated (M @ R @ M)
)
URDF_AXIS_RULE = ("xyz", (1, 2))  # <axis>: same-angle mirror axis = -M @ a
URDF_INERTIA_RULES = (("ixy", (0,)), ("ixz", (0,)))

BODY_RULES = (
    ("pos", (0,)),      # body/geom/joint positions: x -> -x
    ("quat", (2, 3)),   # quaternions (w x y z): y, z -> negated (M @ R @ M)
    ("axis", (1, 2)),   # joint axes: same-angle mirror axis = -M @ a
    ("fullinertia", (3, 4)),  # inertia (ixx iyy izz ixy ixz iyz): ixy, ixz negated
)


def _mirror_urdf_numbers(text: str) -> str:
    out = []
    for line in text.split("\n"):
        stripped = line.lstrip()
        if stripped.startswith("<axis"):
            line = _mirror_attr(line, *URDF_AXIS_RULE)
        elif stripped.startswith("<origin"):
            for attr, idx in URDF_RULES:
                line = _mirror_attr(line, attr, idx)
        elif stripped.startswith("<inertia "):
            for attr, idx in URDF_INERTIA_RULES:
                line = _mirror_attr(line, attr, idx)
        out.append(line)
    return "\n".join(out)


UNMIRRORED = ("fromto", "euler", "xyaxes", "zaxis", "axisangle")   # MJCF attributes the rules above don't cover


def _mirror_body_numbers(text: str) -> str:
    unknown = [attr for attr in UNMIRRORED if f' {attr}="' in text]
    if unknown:
        raise ValueError(f"cannot mirror {unknown}: write positions as pos and orientations as quat")
    out = []
    for line in text.split("\n"):
        for attr, idx in BODY_RULES:
            line = _mirror_attr(line, attr, idx)
        out.append(line)
    return "\n".join(out)


def mirror_urdf(text: str) -> str:
    return _rename(_mirror_urdf_numbers(text))


def mirror_body_xml(text: str) -> str:
    return _rename(_mirror_body_numbers(text))


def mirror_stl(data: bytes, note: str) -> bytes:
    """Mirror a binary STL across x=0: negate x, swap winding, fix normals."""
    n_tri = int(np.frombuffer(data[80:84], dtype="<u4")[0])
    if len(data) != 84 + 50 * n_tri:
        raise ValueError("not a binary STL (or truncated)")
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(data[84:], dtype=dtype).copy()
    tris["normal"][:, 0] *= -1.0          # reflected outward normal = M @ n
    tris["v"][:, :, 0] *= -1.0            # reflect vertices across x = 0
    tris["v"] = tris["v"][:, [0, 2, 1], :]  # invert winding to keep normals outward
    header = note.encode()[:80].ljust(80, b" ")
    return header + np.uint32(n_tri).tobytes() + tris.tobytes()


def generate() -> dict:
    """Return {relative_path: bytes} for every left file derived from right."""
    files = {}
    files[URDF_LEFT] = mirror_urdf(URDF_RIGHT.read_text()).encode()
    files[BODY_LEFT] = mirror_body_xml(BODY_RIGHT.read_text()).encode()
    files[MJCF_LEFT] = _rename(MJCF_RIGHT.read_text()).encode()
    # Every mesh the right URDF or MJCF references.
    meshes = re.findall(r"assets/right/([\w.-]+\.stl)", URDF_RIGHT.read_text() + MJCF_RIGHT.read_text())
    for name in sorted(set(meshes)):
        note = f"mirror(x) of right/{name} by utils/mirror_left_from_right.py"
        files[MESH_LEFT_DIR / _rename(name)] = mirror_stl((MESH_RIGHT_DIR / name).read_bytes(), note)
    return files


def self_test() -> None:
    """The numeric mirror applied twice must reproduce the sources exactly."""
    for path, fn in ((URDF_RIGHT, _mirror_urdf_numbers),
                     (BODY_RIGHT, _mirror_body_numbers)):
        text = path.read_text()
        once = fn(text)
        if once == text:
            raise AssertionError(f"mirror is a no-op on {path.name}")
        if fn(once) != text:
            raise AssertionError(f"double-mirror self-test failed for {path.name}")
    sample = (MESH_RIGHT_DIR / "M-AP.stl").read_bytes()
    twice_stl = mirror_stl(mirror_stl(sample, "tmp"), "tmp")
    if twice_stl[84:] != sample[84:]:
        raise AssertionError("double-mirror self-test failed for STL geometry")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify existing left files match a fresh mirror of right; write nothing",
    )
    args = parser.parse_args(argv)

    self_test()
    files = generate()

    extra = sorted(set(MESH_LEFT_DIR.glob("*.stl")) - set(files))   # left meshes whose right one is gone
    stale = [str(p.relative_to(REPO_ROOT)) for p, data in files.items()
             if not p.exists() or p.read_bytes() != data] + [str(p.relative_to(REPO_ROOT)) for p in extra]
    if args.check:
        if stale:
            print("STALE (left != mirror(right)):")
            for p in stale:
                print(f"  {p}")
            return 1
        print(f"OK: all {len(files)} left files are exact mirrors of right.")
        return 0

    for path, data in files.items():
        path.write_bytes(data)
    for path in extra:
        path.unlink()
    print(f"Wrote {len(files)} left files mirrored from right "
          f"({len(stale) - len(extra)} changed, {len(extra)} removed).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
