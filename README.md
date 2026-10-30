<div style="display: flex; gap: 10px;">
<p align="center">
  <img src=".docs/orcadescription_header.png" alt="ORCA Hand Description Header" style="max-width: 100%; height: auto;" />
</p>

</div>

# Orcahand Description

This repository contains the description files for the Orcahand model (both URDF and MJCF).

Versioned model layouts live under:
- `v1/`: legacy hand description files, including the `extended` variants
- `v2/`: updated hand description files from the newer Fusion360 export

The `extended` version contains additional bodies (incl. inertial properties) such as the camera mount, the U2D2 board and fans.

## v2 names

Every v2 name starts with the side, `right_` or `left_`. Left names are the right names with the prefix swapped.

**Joints** are `{side}_{joint}`, using orca_core's 17 joint ids: `wrist`; `thumb_cmc`, `thumb_abd`, `thumb_mcp`, `thumb_dip`; and `{finger}_abd`, `{finger}_mcp`, `{finger}_pip` for `index`, `middle`, `ring`, `pinky` (e.g. `right_index_mcp`). The URDF also has one fixed joint, `{side}_tower_fixed`.

**Parts** (URDF links and MJCF bodies), from the arm outwards:

| Part | What it is | Moves at joint |
|---|---|---|
| `{side}_forearm` | forearm structure | (root) |
| `{side}_tower` | tower on the forearm, holds the motors | fixed |
| `{side}_carpals` | palm | `wrist` |
| `{side}_{finger}_ap` | finger base, swings sideways | `{finger}_abd` |
| `{side}_{finger}_pp` | first finger segment | `{finger}_mcp` |
| `{side}_{finger}_fingertip_assembly` | last finger segment (there is no DIP joint) | `{finger}_pip` |
| `{side}_thumb_tp` | thumb base | `thumb_cmc` |
| `{side}_thumb_ap` | second thumb piece | `thumb_abd` |
| `{side}_thumb_pp` | first thumb segment | `thumb_mcp` |
| `{side}_thumb_dp` | last thumb segment | `thumb_dip` |

In the URDF, each part's frame sits on the joint it moves at, so the last finger segment's origin is its PIP joint (thumb: DIP joint). In the MJCF, the hand hangs from a `{side}_base` body, and motors are `{side}_{joint}_actuator`.

Mesh files keep their Fusion360 part names (e.g. `R-Carpals.stl`). The ring finger uses the middle finger's `M-` meshes. The left hand is generated from the right by `utils/mirror_left_from_right.py`; `--check` verifies it.

## Example Usage
1. Clone the repository:
   ```bash
   git clone git@github.com:orcahand/orcahand_description.git
   cd orcahand_description
   ```
2. Install the required visualization dependencies:
   ```bash
   pip install mujoco
   ```
3. Simulate any version using MuJoCo:
   ```bash
   python -m mujoco.viewer --mjcf=$(pwd)/v1/scene_combined.xml
   python -m mujoco.viewer --mjcf=$(pwd)/v2/scene_combined.xml
   ```

## Note on Meshes
Visual meshes contain the following amount of faces:
- Main tower, camera & fans: 15'000
- Rest of base: 2'000
- Skin: 5'000
- Rest: 500

Collision meshes contain the following amount of faces:
- Main tower, camera & fans: 7500
- Rest of base: 1000
- Skin: 500
- Rest: 250

You can further reduce or mirror meshes using the `utils/mesh_utils.py` script. With the same script, you can also print their number of faces. Some extra dependencies are required:
```bash
cd utils
pip install -r utils_requirements.txt
```

We can also recommend the VSCode extension `mtsmfm.vscode-stl-viewer` for quickly visualizing STL meshes.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or support, please contact the maintainers of this repo or the Orcahand team via our website ([https://orcahand.com](https://orcahand.com)).
