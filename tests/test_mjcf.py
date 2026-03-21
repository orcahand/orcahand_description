from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")

ROOT_DIR = Path(__file__).resolve().parent.parent

MJCF_ENTRYPOINTS = sorted(
    [
        *ROOT_DIR.glob("v1/models/mjcf/*.mjcf"),
        *ROOT_DIR.glob("v1/scene*.xml"),
        *ROOT_DIR.glob("v2/scene*.xml"),
    ]
)


@pytest.mark.parametrize("mjcf_file", MJCF_ENTRYPOINTS, ids=lambda path: str(path.relative_to(ROOT_DIR)))
def test_mjcf_parsing(mjcf_file: Path) -> None:
    assert mjcf_file.exists(), f"File not found: {mjcf_file}"
    model = mujoco.MjModel.from_xml_path(str(mjcf_file))
    assert model is not None, f"Failed to load MJCF file: {mjcf_file}"
