# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Make the v2 RIGHT hand's collision shapes: convex pieces that follow each part's outer surface, printed part and
skin together. The URDF and the MJCF get the same pieces.

usage: python utils/make_v2_collisions.py

Each part's visual meshes (the URDF's, so the forearm's logo stays out: the real one is engraved) are split into
convex pieces with CoACD (seed 0): one piece per part, except the parts in PIECES (palm, tower and the fingertips,
which touch the most). Each piece keeps at most MAX_VERTICES vertices, which MuJoCo, its GPU backend MJX and Isaac's
importer use as they are. The pieces go to v2/models/assets/right/<part>_collision[_<k>].stl in the part's MJCF body frame (mm, like
the visual meshes). The URDF gets one <collision> per piece, the MJCF one geom per piece in group 3 (hidden in the
MuJoCo viewer by default); the visual geoms stop colliding, and the MJCF excludes the part pairs in EXCLUDE. Run
utils/mirror_left_from_right.py afterwards for the left hand.
"""
import re
from pathlib import Path

import coacd
import trimesh

REPO = Path(__file__).resolve().parents[1]
ASSETS = REPO / "v2" / "models" / "assets" / "right"
URDF = REPO / "v2" / "models" / "urdf" / "orcahand_right.urdf"
MJCF = REPO / "v2" / "models" / "mjcf" / "orcahand_right.mjcf"
BODY = REPO / "v2" / "models" / "mjcf" / "orcahand_right_body.xml"

# part -> (pieces, CoACD concavity threshold, CoACD remeshing resolution); the other parts: ONE_PIECE. The fingertips
# get four pieces on a finer grid: about 1 mm at worst on their skin around the tip and pad (0.4 mm for 95 %), against
# 4.7 mm with one piece. More pieces don't lower the worst case, which the vertex limit sets.
ONE_PIECE = (1, 1.0, 50)
PIECES = {"carpals": (8, 0.05, 50), "tower": (6, 0.05, 50)} | {
    f"{digit}_dp": (4, 0.02, 200) for digit in ("thumb", "index", "middle", "ring", "pinky")}
MAX_VERTICES = 64
# Part pairs whose pieces touch where the real parts don't, found by sampling 20,000 poses within the joint limits:
# the palm and the tower (MuJoCo skips parent-child pairs, but not under a parent fixed to the world), and the palm
# and the first segment after each base joint, whose pieces meet across that joint. Fingers, thumb and palm otherwise
# touch each other as on the real hand.
EXCLUDE = [("tower", "carpals"), ("carpals", "index_pp"), ("carpals", "middle_pp"), ("carpals", "ring_pp"),
           ("carpals", "pinky_pp"), ("carpals", "thumb_ap")]
NUM = "{:.6g}"


def pieces(mesh, count, threshold, resolution):
    """Convex pieces of the mesh, each with at most MAX_VERTICES vertices."""
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=threshold, max_convex_hull=count,
                            preprocess_resolution=resolution, decimate=True, max_ch_vertex=MAX_VERTICES, seed=0)
    return [trimesh.Trimesh(v, f).convex_hull for v, f in parts]


def collision_block(origin, name):
    return (f"\t\t<collision>\n\t\t\t{origin}\n\t\t\t<geometry>\n"
            f'\t\t\t\t<mesh filename="package://orcahand_description/v2/models/assets/right/{name}.stl" scale="0.001 0.001 0.001"/>\n'
            f"\t\t\t</geometry>\n\t\t</collision>\n")


def main():
    coacd.set_log_level("error")
    urdf, assets, body = URDF.read_text(), MJCF.read_text(), BODY.read_text()
    urdf = re.sub(r"\t\t<collision>\n.*?\t\t</collision>\n", "", urdf, flags=re.S)
    assets = re.sub(r'\t\t<mesh name="right_\w+_collision\w*"[^\n]*\n', "", assets)
    body = re.sub(r'\t*<geom mesh="right_\w+_collision\w*"[^\n]*\n', "", body)
    new_assets, new_pieces = "", []
    for block in re.findall(r'\t<link name="right_\w+">\n.*?\t</link>\n', urdf, re.S):
        link = re.match(r'\t<link name="right_(\w+)">', block).group(1)
        files = re.findall(r'assets/right/([\w.-]+)\.stl', block)
        origin = re.search(r"<visual>\s*(<origin [^>]*/>)", block).group(1)
        mesh = trimesh.util.concatenate([trimesh.load(ASSETS / f"{f}.stl") for f in files])
        shapes = pieces(mesh, *PIECES.get(link, ONE_PIECE))
        names = [f"{link}_collision" + (f"_{k}" if len(shapes) > 1 else "") for k in range(len(shapes))]
        for shape, name in zip(shapes, names):
            new_pieces.append((name, shape))
            new_assets += f'\t\t<mesh name="right_{name}" file="models/assets/right/{name}.stl" />\n'
        print(f"  {link}: {len(shapes)} piece(s), at most {max(len(s.vertices) for s in shapes)} vertices")
        last_visual = block.rindex("\t\t</visual>\n") + len("\t\t</visual>\n")
        urdf = urdf.replace(block, block[:last_visual] + "".join(collision_block(origin, n) for n in names) + block[last_visual:])
        # MJCF: the body's own geoms stop colliding; the pieces go in before the body closes
        start = body.index(f'<body name="right_{link}"')
        indent = re.search(r"\t*$", body[:start]).group(0)
        end = body.index(f"\n{indent}</body>", start)
        own = body[start:end]
        own = re.sub(rf'(\n{indent}\t<geom (?:(?!contype)[^\n])*?) />', r'\1 contype="0" conaffinity="0" />', own)
        geoms = "".join(f'\n{indent}\t<geom mesh="right_{n}" group="3" />' for n in names)
        body = body[:start] + own + geoms + body[end:]
    last_mesh = assets.rindex("\t\t<mesh ")
    last_mesh = assets.index("\n", last_mesh) + 1
    assets = assets[:last_mesh] + new_assets + assets[last_mesh:]
    contact = "".join(f'\t\t<exclude body1="right_{a}" body2="right_{b}" />\n' for a, b in EXCLUDE)
    assets = re.sub(r"\t<contact>\n.*?\t</contact>\n", lambda _: f"\t<contact>\n{contact}\t</contact>\n" if contact else "",
                    assets, flags=re.S)
    # the old pieces go only once every new one is made, so a run that fails midway leaves the files as they were
    for old in ASSETS.glob("*_collision*.stl"):
        old.unlink()
    for name, shape in new_pieces:
        shape.export(ASSETS / f"{name}.stl")
    URDF.write_text(urdf)
    MJCF.write_text(assets)
    BODY.write_text(body)
    print("done: now run python utils/mirror_left_from_right.py")


if __name__ == "__main__":
    main()
