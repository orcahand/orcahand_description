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

The v2 names changed in November 2026 ([#20](https://github.com/orcahand/orcahand_description/pull/20)) from Fusion360 export names such as `P-AP_f5e42b61` to the names below. If your code uses the old names, see [Renaming your files](#renaming-your-files).

Every v2 name starts with the side, `right_` or `left_`. Left names are the right names with the prefix swapped.

**Joints** are `{side}_{joint}`, using orca_core's 17 joint ids: `wrist`; `thumb_cmc`, `thumb_abd`, `thumb_mcp`, `thumb_dip`; and `{finger}_abd`, `{finger}_mcp`, `{finger}_pip` for `index`, `middle`, `ring`, `pinky` (e.g. `right_index_mcp`). The URDF also has one fixed joint, `{side}_tower_fixed`. Positive finger spread (`{finger}_abd`) moves the finger toward the thumb, in this model and on the real hand.

**Parts** (URDF links and MJCF bodies), from the arm outwards:

| Part | What it is | Moves at joint |
|---|---|---|
| `{side}_forearm` | forearm structure, holds the 16 finger motors and the board | (root) |
| `{side}_tower` | tower on the forearm, holds the wrist motor | fixed |
| `{side}_carpals` | palm | `wrist` |
| `{side}_{finger}_ap` | finger base, swings sideways | `{finger}_abd` |
| `{side}_{finger}_pp` | first finger segment | `{finger}_mcp` |
| `{side}_{finger}_dp` | last finger segment, CAD part `I-IP`/`M-IP`/`P-IP` (there is no DIP joint) | `{finger}_pip` |
| `{side}_thumb_tp` | thumb base | `thumb_cmc` |
| `{side}_thumb_ap` | second thumb piece | `thumb_abd` |
| `{side}_thumb_pp` | first thumb segment | `thumb_mcp` |
| `{side}_thumb_dp` | last thumb segment | `thumb_dip` |

In the URDF, each part's frame sits on the joint it moves at, so the last finger segment's origin is its PIP joint (thumb: DIP joint). In the MJCF, the hand hangs from a `{side}_base` body, and motors are `{side}_{joint}_actuator`. Each MJCF body frame sits at its part's mesh origin: the URDF link frame moved by that link's visual origin, with the same orientation.

**Meshes** show each part's printed body in dark grey and its silicone skin in white (the palm skin in black), within about 0.1 mm of the CAD. Screws, bearings, motors and electronics are not drawn. The forearm is its convex hull: closed, but flat over the fan grilles and the connector box. Mesh files keep their CAD part names; the ring finger uses the middle finger's `M-` meshes. The left hand is generated from the right by `utils/mirror_left_from_right.py`; `--check` verifies it.

### Renaming your files

`v2/name_changes.csv` lists every old name with its new name. An empty new name means the name was removed, with no replacement. The list covers two sets of old names:
- `--source main`: orcahand_description `main` before the rename, e.g. `P-AP_f5e42b61` or `right_i-mcp`.
- `--source fb-split-mjcf`: the `fb/split-mjcf-scene-positioning` branch, e.g. `right_index_fingertip_assembly`.

`utils/migrate_v2_names.py` applies the list to your own files:
1. See what would change. This writes nothing:
   ```bash
   python utils/migrate_v2_names.py --source main --side right FILE...
   ```
2. Commit your files, then rewrite them:
   ```bash
   python utils/migrate_v2_names.py --source main --side right --write FILE...
   ```

Run it on one hand's files at a time: some old names were the same in the right and left files. The script skips a file with the other hand's or the other source's old names, and in both modes exits with code 1 if it skipped a file you need to check. Names without the side prefix, as in orca_ui's bundle, are not covered.

## v2 kinematics and masses

**Joints** follow the v2.1 CAD assembly (November 2026). At 0° the palm is in line with the forearm.

**Centres of mass and inertias** were computed from each part's bone and skin meshes, assuming uniform density, and then scaled to the part's mass. The URDF and the MJCF have the same masses, centres of mass and inertias.

**Joint damping, friction and armature** are only in the MJCF: damping 0.1 N·m·s/rad, friction loss 0.001 N·m, armature 0.001 kg·m². These values are estimates, not measurements. The URDF has no joint damping, friction or armature: URDF can't express armature, and damping without armature makes simulators that apply damping explicitly unstable.

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
