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


@pytest.mark.parametrize("side", SIDES)
def test_names(side: str) -> None:
    urdf = ET.parse(urdf_path(side)).getroot()
    joints = urdf.findall("joint")
    assert {link.get("name") for link in urdf.findall("link")} == {f"{side}_{p}" for p in PARTS}
    assert {j.get("name") for j in joints if j.get("type") == "revolute"} == {f"{side}_{j}" for j in JOINTS}
    assert {j.get("name") for j in joints if j.get("type") == "fixed"} == {f"{side}_tower_fixed"}

    model = load_mjcf(side)
    bodies = {model.body(i).name for i in range(model.nbody)}
    assert {f"{side}_{p}" for p in PARTS} | {f"{side}_base"} <= bodies
    assert {model.joint(i).name for i in range(model.njnt)} == {f"{side}_{j}" for j in JOINTS}
    assert {model.actuator(i).name for i in range(model.nu)} == {f"{side}_{j}_actuator" for j in JOINTS}


def test_left_is_mirror_of_right() -> None:
    """The left files must be regenerated from the right ones after every change."""
    script = ROOT_DIR / "utils" / "mirror_left_from_right.py"
    result = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


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


@pytest.mark.parametrize("side", SIDES)
def test_urdf_and_mjcf_agree(side: str) -> None:
    """Both files give the same joints, joint limits, masses, centres of mass and inertias."""
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


@pytest.mark.parametrize("side", SIDES)
def test_masses_are_plausible(side: str) -> None:
    model = load_mjcf(side)
    bodies = {part: model.body(f"{side}_{part}") for part in PARTS}
    masses = {part: body.mass[0] for part, body in bodies.items()}
    assert 1.0 < sum(masses.values()) < 1.4
    assert 0.2 < sum(m for part, m in masses.items() if part not in ("forearm", "tower")) < 0.26
    for part, body in bodies.items():
        radius = np.sqrt(body.inertia / body.mass[0])
        assert np.all((radius > 0.005) & (radius < 0.06)), part


@pytest.mark.parametrize("side", SIDES)
def test_mjcf_holds_still_under_gravity(side: str) -> None:
    """With every actuator holding 0, the hand settles instead of drifting or blowing up."""
    model = load_mjcf(side)
    data = mujoco.MjData(model)
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.max(np.abs(data.qvel)) < 0.05
