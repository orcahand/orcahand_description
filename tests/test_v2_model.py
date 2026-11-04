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


def load_urdf(side: str):
    """Load a v2 URDF into MuJoCo, keeping every link as its own body."""
    text = (V2 / "models" / "urdf" / f"orcahand_{side}.urdf").read_text()
    text = text.replace("package://orcahand_description/", f"{ROOT_DIR}/")
    text = text.replace(
        "</robot>",
        '<mujoco><compiler discardvisual="false" fusestatic="false" strippath="false"/></mujoco></robot>',
    )
    return mujoco.MjModel.from_xml_string(text)


def load_mjcf(side: str):
    return mujoco.MjModel.from_xml_path(str(V2 / f"scene_{side}.xml"))


def pose(data, body_id: int) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = data.xmat[body_id].reshape(3, 3)
    transform[:3, 3] = data.xpos[body_id]
    return transform


@pytest.mark.parametrize("side", SIDES)
def test_names(side: str) -> None:
    urdf = ET.parse(V2 / "models" / "urdf" / f"orcahand_{side}.urdf").getroot()
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
        abd = model.joint(f"{side}_{finger}_abd").id
        mcp = model.joint(f"{side}_{finger}_mcp").id
        normal = np.cross(data.xaxis[abd], data.xaxis[mcp])
        normal /= np.linalg.norm(normal)
        distance_mm = abs(normal @ (data.xanchor[mcp] - data.xanchor[abd])) * 1e3
        assert distance_mm == pytest.approx(spacing_mm, abs=1e-3), finger


@pytest.mark.parametrize("side", SIDES)
def test_urdf_and_mjcf_agree(side: str) -> None:
    """Both files give the same joints, masses, centres of mass and inertias."""
    model_u, model_m = load_urdf(side), load_mjcf(side)
    data_u, data_m = mujoco.MjData(model_u), mujoco.MjData(model_m)
    mujoco.mj_forward(model_u, data_u)
    mujoco.mj_forward(model_m, data_m)

    # MJCF world -> URDF world, through the forearm, whose frame is the same in both.
    root = f"{side}_forearm"
    align = pose(data_u, model_u.body(root).id) @ np.linalg.inv(pose(data_m, model_m.body(root).id))
    rot, shift = align[:3, :3], align[:3, 3]

    for joint in JOINTS:
        ju, jm = model_u.joint(f"{side}_{joint}").id, model_m.joint(f"{side}_{joint}").id
        np.testing.assert_allclose(data_u.xanchor[ju], rot @ data_m.xanchor[jm] + shift, atol=1e-6)
        np.testing.assert_allclose(data_u.xaxis[ju], rot @ data_m.xaxis[jm], atol=1e-6)

    for part in PARTS:
        bu, bm = model_u.body(f"{side}_{part}").id, model_m.body(f"{side}_{part}").id
        assert model_u.body_mass[bu] == pytest.approx(model_m.body_mass[bm], rel=1e-6), part
        np.testing.assert_allclose(data_u.xipos[bu], rot @ data_m.xipos[bm] + shift, atol=1e-6)
        inertia_u = data_u.ximat[bu].reshape(3, 3) @ np.diag(model_u.body_inertia[bu]) @ data_u.ximat[bu].reshape(3, 3).T
        frame_m = rot @ data_m.ximat[bm].reshape(3, 3)
        inertia_m = frame_m @ np.diag(model_m.body_inertia[bm]) @ frame_m.T
        np.testing.assert_allclose(inertia_u, inertia_m, atol=1e-6 * np.abs(inertia_u).max())


@pytest.mark.parametrize("side", SIDES)
def test_masses_are_plausible(side: str) -> None:
    model = load_mjcf(side)
    masses = {part: model.body_mass[model.body(f"{side}_{part}").id] for part in PARTS}
    assert 1.0 < sum(masses.values()) < 1.4
    assert 0.2 < sum(m for part, m in masses.items() if part not in ("forearm", "tower")) < 0.26
    for part in PARTS:
        body = model.body(f"{side}_{part}").id
        radius = np.sqrt(model.body_inertia[body] / model.body_mass[body])
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
