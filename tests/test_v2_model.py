"""Checks on the v2 hand models: names, kinematics, masses and simulation."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT_DIR = Path(__file__).resolve().parent.parent
V2 = ROOT_DIR / "v2"
SIDES = ("right", "left")
FINGERS = ("index", "middle", "ring", "pinky")

# orca_core's 17 joint ids.
JOINTS = ["wrist", "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"] + [
    f"{finger}_{joint}" for finger in FINGERS for joint in ("abd", "mcp", "pip")
]
PARTS = ["forearm", "tower", "carpals", "thumb_tp", "thumb_ap", "thumb_pp", "thumb_dp"] + [
    f"{finger}_{part}" for finger in FINGERS for part in ("ap", "pp", "dp")
]
# Fingertip points: URDF links on fixed joints, MJCF sites, both on the last segment.
POINTS = [f"{digit}_{point}" for digit in ("thumb",) + FINGERS for point in ("tip", "pad")]

# Distance between each AP part's spread axis and its MCP axis, from the v2.1 CAD
# (orcahand_hardware Orcahand_BaseV2_Right.zip); the v2.0 parts are identical.
AP_SPACING_MM = {"index": 18.75, "middle": 19.25, "ring": 19.25, "pinky": 17.75}


def urdf_path(side: str) -> Path:
    return V2 / "models" / "urdf" / f"orcahand_{side}.urdf"


def load_urdf(side: str):
    """Load a v2 URDF into MuJoCo, keeping every link as its own body."""
    text = urdf_path(side).read_text()
    text = text.replace("package://orcahand_description/", f"{ROOT_DIR}/")
    text = text.replace(
        "</robot>",
        '<mujoco><compiler discardvisual="false" fusestatic="false" strippath="false"/></mujoco></robot>',
    )
    return mujoco.MjModel.from_xml_string(text)


def load_mjcf(side: str):
    return mujoco.MjModel.from_xml_path(str(V2 / f"scene_{side}.xml"))


def collision_pieces(model, data, side: str) -> dict:
    """Mesh name -> world vertices of every colliding mesh geom of the hand."""
    pieces = {}
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        assert model.body(model.geom_bodyid[g]).name.startswith(side)
        mesh = model.geom_dataid[g]
        vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        pieces[model.mesh(mesh).name] = data.geom_xpos[g] + vertices @ data.geom_xmat[g].reshape(3, 3).T
    return pieces


@pytest.mark.parametrize("side", SIDES)
def test_names(side: str) -> None:
    urdf = ET.parse(urdf_path(side)).getroot()
    joints = urdf.findall("joint")
    assert {link.get("name") for link in urdf.findall("link")} == {f"{side}_{p}" for p in PARTS + POINTS}
    assert {j.get("name") for j in joints if j.get("type") == "revolute"} == {f"{side}_{j}" for j in JOINTS}
    assert {j.get("name") for j in joints if j.get("type") == "fixed"} == {
        f"{side}_{name}_fixed" for name in ["tower"] + POINTS
    }

    model = load_mjcf(side)
    bodies = {model.body(i).name for i in range(model.nbody)}
    assert {f"{side}_{p}" for p in PARTS} | {f"{side}_base"} <= bodies
    assert {model.joint(i).name for i in range(model.njnt)} == {f"{side}_{j}" for j in JOINTS}
    assert {model.actuator(i).name for i in range(model.nu)} == {f"{side}_{j}_actuator" for j in JOINTS}
    assert {model.site(i).name for i in range(model.nsite)} == {f"{side}_{p}" for p in POINTS}
    parents = {j.get("name"): j.find("parent").get("link") for j in joints}
    for point in POINTS:
        segment = f"{side}_{point.split('_')[0]}_dp"
        assert parents[f"{side}_{point}_fixed"] == segment
        assert model.body(model.site(f"{side}_{point}").bodyid[0]).name == segment


def test_left_is_mirror_of_right() -> None:
    """The left files must be regenerated from the right ones after every change."""
    script = ROOT_DIR / "utils" / "mirror_left_from_right.py"
    result = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("side", SIDES)
def test_meshes_load_in_mujoco(side: str) -> None:
    """MuJoCo refuses binary STL files with more than 200,000 triangles; a finer mesh must stay below."""
    for stl in sorted((V2 / "models" / "assets" / side).glob("*.stl")):
        data = stl.read_bytes()
        triangles = int(np.frombuffer(data[80:84], dtype="<u4")[0])
        assert len(data) == 84 + 50 * triangles, f"{stl.name}: not a binary STL"
        assert 0 < triangles <= 200_000, f"{stl.name}: {triangles} triangles"


@pytest.mark.parametrize("loader", [load_urdf, load_mjcf], ids=["urdf", "mjcf"])
@pytest.mark.parametrize("side", SIDES)
def test_ap_spacing_matches_cad(side: str, loader) -> None:
    model = loader(side)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for finger, spacing_mm in AP_SPACING_MM.items():
        abd, mcp = data.joint(f"{side}_{finger}_abd"), data.joint(f"{side}_{finger}_mcp")
        normal = np.cross(abd.xaxis, mcp.xaxis)
        normal /= np.linalg.norm(normal)
        distance_mm = abs(normal @ (mcp.xanchor - abd.xanchor)) * 1e3
        assert distance_mm == pytest.approx(spacing_mm, abs=1e-3), finger


@pytest.mark.parametrize("loader", [load_urdf, load_mjcf], ids=["urdf", "mjcf"])
@pytest.mark.parametrize("side", SIDES)
def test_positive_spread_moves_toward_the_thumb(side: str, loader) -> None:
    """+15° of each {finger}_abd moves that finger's PIP joint toward the thumb side."""
    model = loader(side)
    data = mujoco.MjData(model)

    def pip_positions(joint: str = "") -> dict:
        data.qpos[:] = 0
        if joint:
            data.qpos[model.joint(joint).qposadr[0]] = np.deg2rad(15)
        mujoco.mj_kinematics(model, data)
        return {finger: data.joint(f"{side}_{finger}_pip").xanchor.copy() for finger in FINGERS}

    rest = pip_positions()
    toward_thumb = rest["index"] - rest["pinky"]
    toward_thumb /= np.linalg.norm(toward_thumb)
    for finger in FINGERS:
        moved = pip_positions(f"{side}_{finger}_abd")[finger]
        assert (moved - rest[finger]) @ toward_thumb > 0.005, finger


@pytest.mark.parametrize("side", SIDES)
def test_urdf_and_mjcf_agree(side: str) -> None:
    """Both files give the same joints, joint limits, masses, centres of mass, inertias, fingertip points and
    collision pieces."""
    model_u, model_m = load_urdf(side), load_mjcf(side)
    data_u, data_m = mujoco.MjData(model_u), mujoco.MjData(model_m)
    mujoco.mj_forward(model_u, data_u)
    mujoco.mj_forward(model_m, data_m)

    # MJCF world -> URDF world, through the forearm, whose frame is the same in both.
    root_u, root_m = data_u.body(f"{side}_forearm"), data_m.body(f"{side}_forearm")
    rot = root_u.xmat.reshape(3, 3) @ root_m.xmat.reshape(3, 3).T
    shift = root_u.xpos - rot @ root_m.xpos

    for joint in JOINTS:
        name = f"{side}_{joint}"
        ju, jm = data_u.joint(name), data_m.joint(name)
        np.testing.assert_allclose(ju.xanchor, rot @ jm.xanchor + shift, atol=1e-6)
        np.testing.assert_allclose(ju.xaxis, rot @ jm.xaxis, atol=1e-6)
        limits = model_m.joint(name).range
        np.testing.assert_allclose(model_u.joint(name).range, limits, atol=1e-6, err_msg=joint)
        np.testing.assert_allclose(model_m.actuator(f"{name}_actuator").ctrlrange, limits, atol=1e-6, err_msg=joint)

    for part in PARTS:
        name = f"{side}_{part}"
        bu, bm = data_u.body(name), data_m.body(name)
        assert model_u.body(name).mass[0] == pytest.approx(model_m.body(name).mass[0], rel=1e-6), part
        np.testing.assert_allclose(bu.xipos, rot @ bm.xipos + shift, atol=1e-6)
        frame_u = bu.ximat.reshape(3, 3)
        frame_m = rot @ bm.ximat.reshape(3, 3)
        inertia_u = frame_u @ np.diag(model_u.body(name).inertia) @ frame_u.T
        inertia_m = frame_m @ np.diag(model_m.body(name).inertia) @ frame_m.T
        np.testing.assert_allclose(inertia_u, inertia_m, atol=1e-6 * np.abs(inertia_u).max())

    for point in POINTS:
        name = f"{side}_{point}"
        link, site = data_u.body(name), data_m.site(name)
        np.testing.assert_allclose(link.xpos, rot @ site.xpos + shift, atol=1e-6, err_msg=point)
        np.testing.assert_allclose(link.xmat.reshape(3, 3), rot @ site.xmat.reshape(3, 3), atol=1e-5, err_msg=point)

    pieces_u, pieces_m = collision_pieces(model_u, data_u, side), collision_pieces(model_m, data_m, side)
    assert pieces_u.keys() == {name.removeprefix(f"{side}_") for name in pieces_m}
    for name, points in pieces_u.items():
        np.testing.assert_allclose(points, pieces_m[f"{side}_{name}"] @ rot.T + shift, atol=1e-6, err_msg=name)


@pytest.mark.parametrize("side", SIDES)
def test_masses_are_plausible(side: str) -> None:
    """The weighed v2.1.1 right hand (Dynamixel): 1125.3 g, of which 230.15 g move. (MuJoCo refuses impossible inertias.)"""
    model = load_mjcf(side)
    bodies = {part: model.body(f"{side}_{part}") for part in PARTS}
    masses = {part: body.mass[0] for part, body in bodies.items()}
    assert sum(masses.values()) == pytest.approx(1.1253, abs=1e-6)
    assert sum(m for part, m in masses.items() if part not in ("forearm", "tower")) == pytest.approx(0.23015, abs=1e-6)
    for part, body in bodies.items():
        radius = np.sqrt(body.inertia / body.mass[0])
        assert np.all((radius > 0.005) & (radius < 0.06)), part


@pytest.mark.parametrize("side", SIDES)
def test_centres_of_mass_lie_in_their_parts(side: str) -> None:
    """Each part's centre of mass lies inside the box around its meshes; a frame mix-up would put it outside."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for part in PARTS:
        body = model.body(f"{side}_{part}").id
        points = []
        for g in np.flatnonzero((model.geom_bodyid == body) & (model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)):
            mesh = model.geom_dataid[g]
            vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
            points.append(data.geom_xpos[g] + vertices @ data.geom_xmat[g].reshape(3, 3).T)
        points = np.vstack(points)
        assert np.all(points.min(0) < data.xipos[body]) and np.all(data.xipos[body] < points.max(0)), part


@pytest.mark.parametrize("side", SIDES)
def test_mjcf_holds_still_under_gravity(side: str) -> None:
    """With every actuator holding 0, the hand settles instead of drifting or blowing up."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.max(np.abs(data.qvel)) < 0.05


@pytest.mark.parametrize("side", SIDES)
def test_fingertip_points_sit_on_the_skin(side: str) -> None:
    """Each tip and pad lies on its last segment's skin, within 0.15 mm along its Z axis, and Z points out of the skin."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for point in POINTS:
        site = data.site(f"{side}_{point}")
        body = model.site(f"{side}_{point}").bodyid[0]
        skin = next(g for g in np.flatnonzero(model.geom_bodyid == body)
                    if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and "Skin" in model.mesh(model.geom_dataid[g]).name)
        z = site.xmat.reshape(3, 3)[:, 2]
        from_outside = mujoco.mj_rayMesh(model, data, skin, site.xpos + 0.001 * z, -z)
        assert from_outside == pytest.approx(0.001, abs=0.00015), point
        assert mujoco.mj_rayMesh(model, data, skin, site.xpos + 0.0002 * z, z) == -1, point   # nothing of the skin outside


@pytest.mark.parametrize("side", SIDES)
def test_collision_pieces_are_light_and_clear(side: str) -> None:
    """Only the collision pieces collide: each part has its own, every piece is convex with at most 64 vertices and
    sits in geom group 3; and no two parts touch at 0 deg."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hand = {g for g in range(model.ngeom) if model.body(model.geom_bodyid[g]).name.startswith(side)}
    colliding = [g for g in hand if model.geom_contype[g] or model.geom_conaffinity[g]]
    assert {model.body(model.geom_bodyid[g]).name for g in colliding} == {f"{side}_{p}" for p in PARTS}
    for g in colliding:
        mesh = model.geom_dataid[g]
        name = model.mesh(mesh).name
        assert model.geom_group[g] == 3 and "_collision" in name, name
        vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        faces = model.mesh_face[model.mesh_faceadr[mesh]:model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]]
        assert len(vertices) <= 64, name
        a, b, c = (vertices[faces[:, k]] for k in range(3))
        normals = np.cross(b - a, c - a)
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        heights = normals @ vertices.T - np.einsum("fk,fk->f", normals, a)[:, None]   # every vertex below every face
        assert heights.max() < 2e-5, name   # STL stores float32: nearly flat faces tilt by micrometres
    touching = [(model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]) for c in data.contact[:data.ncon]
                if c.geom1 in hand and c.geom2 in hand]
    assert not touching, [(model.body(a).name, model.body(b).name) for a, b in touching]


@pytest.mark.parametrize("side", SIDES)
def test_one_joint_at_a_limit_touches_only_a_neighbouring_finger(side: str) -> None:
    """With one joint at either limit and the rest at 0, a finger may touch the neighbour it spreads into; the palm,
    tower and thumb touch nothing (each missing <exclude> shows up here)."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    for j in np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE):
        for limit in model.jnt_range[j]:
            data.qpos[:] = 0
            data.qpos[model.jnt_qposadr[j]] = limit
            mujoco.mj_forward(model, data)
            for c in data.contact[:data.ncon]:
                names = [model.body(model.geom_bodyid[g]).name for g in (c.geom1, c.geom2)]
                digits = sorted(FINGERS.index(n.split("_")[1]) if n.split("_")[1] in FINGERS else -9 for n in names)
                assert digits[1] - digits[0] == 1, (model.joint(j).name, np.degrees(limit), names)
