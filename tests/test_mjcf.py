import os
import mujoco
import pytest

MJCF_DIR = os.path.dirname(os.path.dirname(__file__))

def get_xml_files(directory):
    """
    Get all XML files from the given directory, excluding 'package.xml'.
    """
    xml_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".xml") and file != "package.xml":
                xml_files.append(os.path.join(root, file))
    return xml_files

XML_FILES = get_xml_files(MJCF_DIR)

@pytest.mark.parametrize("xml_file", XML_FILES)
def test_mjcf_parsing(xml_file):
    """
    Test loading MJCF files to ensure there are no parsing errors.
    """
    file_path = os.path.join(MJCF_DIR, xml_file)
    assert os.path.exists(file_path), f"File not found: {file_path}"

    try:
        model = mujoco.MjModel.from_xml_path(file_path)
        assert model is not None, f"Failed to load MJCF file: {file_path}"
    except Exception as e:
        pytest.fail(f"Error parsing MJCF file {file_path}: {e}")


if __name__ == "__main__":
    pytest.main()