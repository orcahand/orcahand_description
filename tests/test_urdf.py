import os
import pytest
from urdf_parser_py.urdf import URDF

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
URDF_DIR = os.path.join(ROOT_DIR, "models", "urdf")

def get_urdf_files(directory):
    """
    Get all URDF files from the given directory, excluding 'package.xml'.
    """
    print(f"Searching for URDF files in: {directory}")  # Debugging
    urdf_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".urdf"):
                urdf_files.append(os.path.join(root, file))
    print(f"Found URDF files: {urdf_files}")  # Debugging
    return urdf_files

URDF_FILES = get_urdf_files(URDF_DIR)

@pytest.mark.parametrize("urdf_file", URDF_FILES)
def test_urdf_parsing(urdf_file):
    """
    Test loading URDF files to ensure there are no parsing errors.
    """
    assert os.path.exists(urdf_file), f"File not found: {urdf_file}"

    try:
        robot = URDF.from_xml_file(urdf_file)
        assert robot is not None, f"Failed to load URDF file: {urdf_file}"
        assert len(robot.links) >= 10, f"URDF file {urdf_file} has fewer than 10 links."

    except Exception as e:
        pytest.fail(f"Error parsing URDF file {urdf_file}: {e}")

if __name__ == "__main__":
    pytest.main()