"""Checks on utils/migrate_v2_names.py, on small files made of real old names."""

from pathlib import Path
import sys

import pytest

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR / "utils"))
import migrate_v2_names  # noqa: E402

# Old main URDF names: the carpals and index AP are each hand's own, the pinky AP
# is one of the 18 names both hands used.
RIGHT = """<robot>
  <link name="R-Carpals_8d1f1041"/>
  <link name="I-AP-R_d95d02d1"/>
  <link name="P-AP_f5e42b61"/>
  <joint name="I-AP-R_d95d02d1_to_R-Carpals_8d1f1041"/>
</robot>
"""
RIGHT_NEW = """<robot>
  <link name="right_carpals"/>
  <link name="right_index_ap"/>
  <link name="right_pinky_ap"/>
  <joint name="right_index_abd"/>
</robot>
"""
LEFT = RIGHT.replace("R-Carpals_8d1f1041", "L-Carpals_719fff8c").replace("I-AP-R_d95d02d1", "I-AP-L_57ce92f7")
# Old main MJCF: right_tower is an old name (now right_base) and a current one.
MJCF = '<body name="right_tower">\n  <body name="right_TopTower-Model_4a80d30e"/>\n</body>\n'
MJCF_NEW = '<body name="right_base">\n  <body name="right_tower"/>\n</body>\n'
# One entry per hand, as in a config file.
BOTH_HANDS = """right:
  base_link: ForeArmStructure-Model_e18f2368
  index_mcp: right_i-mcp
left:
  base_link: ForeArmStructure-Model_e18f2368
  index_mcp: left_i-mcp
"""


def migrate(side: str, *files: Path, write: bool = False, source: str = "main") -> int:
    args = ["--source", source, "--side", side, *map(str, files)]
    return migrate_v2_names.main(args + ["--write"] * write)


@pytest.mark.parametrize("old, new", [(RIGHT, RIGHT_NEW), (MJCF, MJCF_NEW)], ids=["urdf", "mjcf"])
@pytest.mark.parametrize("newline", ["\n", "\r\n"], ids=["lf", "crlf"])
def test_renames_once_and_keeps_line_endings(tmp_path: Path, old: str, new: str, newline: str) -> None:
    path = tmp_path / "hand.xml"
    path.write_bytes(old.replace("\n", newline).encode())
    assert migrate("right", path) == 1  # only reports
    assert path.read_bytes() == old.replace("\n", newline).encode()
    assert migrate("right", path, write=True) == 0
    assert path.read_bytes() == new.replace("\n", newline).encode()
    # A second run finds nothing to do and changes nothing.
    assert migrate("right", path) == 0
    assert migrate("right", path, write=True) == 0
    assert path.read_bytes() == new.replace("\n", newline).encode()


@pytest.mark.parametrize(
    "source, side, text",
    [
        ("main", "right", LEFT),
        ("main", "left", RIGHT),
        ("main", "right", BOTH_HANDS),
        ("main", "left", BOTH_HANDS),
        ("main", "right", RIGHT + LEFT),
        ("fb-split-mjcf", "left", '<body name="right_index_fingertip_assembly"/>\n'),
    ],
    ids=["left-as-right", "right-as-left", "both-as-right", "both-as-left", "both-urdf", "fb-split"],
)
@pytest.mark.parametrize("write", [False, True], ids=["report", "write"])
def test_skips_files_of_the_other_hand(
    tmp_path: Path, capsys: pytest.CaptureFixture, source: str, side: str, text: str, write: bool
) -> None:
    path = tmp_path / "hand.yaml"
    path.write_text(text)
    assert migrate(side, path, write=write, source=source) == 1
    assert path.read_text() == text
    assert "skipped" in capsys.readouterr().out


@pytest.mark.parametrize(
    "source, text",
    [
        ("fb-split-mjcf", RIGHT),
        ("main", '<body name="right_index_fingertip_assembly"/>\n'),
        # As in orca_ui's joint_map.yaml: fb-split names next to names that are current.
        ("main", "links:\n  right_carpals: carpals\n  right_index_fingertip_assembly: index_fingertip_assembly\n"),
    ],
    ids=["main-file", "fb-split-file", "fb-split-file-with-current-names"],
)
@pytest.mark.parametrize("write", [False, True], ids=["report", "write"])
def test_flags_files_of_the_other_source(
    tmp_path: Path, capsys: pytest.CaptureFixture, source: str, text: str, write: bool
) -> None:
    path = tmp_path / "hand.xml"
    path.write_text(text)
    assert migrate("right", path, write=write, source=source) == 1
    assert path.read_text() == text
    assert "use --source" in capsys.readouterr().out


@pytest.mark.parametrize(
    "content",
    [(RIGHT + '<link name="right_pinky_dp"/>\n').encode(), b'<body name="right_tower"/>\n', b"\xff\xfe\x00 not text"],
    ids=["mixed", "only-right_tower", "binary"],
)
@pytest.mark.parametrize("write", [False, True], ids=["report", "write"])
def test_skipped_file_exits_1(tmp_path: Path, content: bytes, write: bool) -> None:
    path = tmp_path / "skipped.xml"
    path.write_bytes(content)
    assert migrate("right", path, write=write) == 1
    assert path.read_bytes() == content


def test_skipped_file_does_not_stop_the_others(tmp_path: Path) -> None:
    skipped, hand = tmp_path / "skipped.stl", tmp_path / "hand.urdf"
    skipped.write_bytes(b"\xff\xfe\x00 not text")
    hand.write_text(RIGHT)
    assert migrate("right", skipped, hand, write=True) == 1
    assert hand.read_text() == RIGHT_NEW


def test_removed_names_stay_and_keep_the_report_failing(tmp_path: Path) -> None:
    path = tmp_path / "hand.xml"
    path.write_text('<joint name="right_i-mcp"/>\n<mesh name="right_I-PP"/>\n')
    assert migrate("right", path, write=True) == 0
    assert path.read_text() == '<joint name="right_index_mcp"/>\n<mesh name="right_I-PP"/>\n'
    assert migrate("right", path) == 1


def test_file_with_only_shared_names_follows_side(tmp_path: Path, capsys: pytest.CaptureFixture) -> None:
    path = tmp_path / "hand.yaml"
    path.write_text("base_link: ForeArmStructure-Model_e18f2368\n")
    assert migrate("left", path, write=True) == 0
    assert path.read_text() == "base_link: left_forearm\n"
    assert "check it is a left hand file" in capsys.readouterr().out
