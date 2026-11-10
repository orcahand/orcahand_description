<div style="display: flex; gap: 10px;">
<p align="center">
  <img src=".docs/orcadescription_header.png" alt="ORCA Hand Description Header" style="max-width: 100%; height: auto;" />
</p>

</div>

# Orcahand Description

This repository contains the description files for the Orcahand model (both URDF and MJCF).

Versioned model layouts live under:
- `v1/`: legacy hand description files, including the `extended` variants
- `v2/`: the v2 hand description files

The `extended` version contains additional bodies (incl. inertial properties) such as the camera mount, the U2D2 board and fans.

## v2 names

The v2 names changed in November 2026 ([#20](https://github.com/orcahand/orcahand_description/pull/20)). If your code uses the old names, see [Renaming your files](#renaming-your-files).

Every v2 name starts with the side, `right_` or `left_`. **Joints** are `{side}_{joint}`, using orca_core's 17 joint ids: `wrist`; `thumb_cmc`, `thumb_abd`, `thumb_mcp`, `thumb_dip`; and `{finger}_abd`, `{finger}_mcp`, `{finger}_pip` for `index`, `middle`, `ring`, `pinky` (e.g. `right_index_mcp`). Positive finger spread (`{finger}_abd`) moves the finger toward the thumb. MJCF motors are `{side}_{joint}_actuator`.

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

In the URDF, each part's frame sits on the joint it moves at. In the MJCF, the hand hangs from a `{side}_base` body, and each body frame sits at its part's mesh origin.

**Fingertip points:** each digit's last segment has two frames on its skin, `{side}_{digit}_tip` at the end of the fingertip and `{side}_{digit}_pad` in the middle of the pad, where pinches touch (`digit` is `thumb`, `index`, `middle`, `ring` or `pinky`). In the URDF they are links on the fixed joints `{side}_{digit}_tip_fixed` and `{side}_{digit}_pad_fixed`; in the MJCF they are sites. Z points out of the skin, X along the segment's joint axis, and Y = Z × X. They were placed by hand on the right hand's skin; the left hand's are their mirror.

**Meshes** show each part's printed body in dark grey and its silicone skin in white (the palm skin in black), within about 0.1 mm of the CAD. Screws, bearings, motors and electronics are not drawn. The forearm is its convex hull: closed, but flat over the fan grilles and the connector box. Mesh files keep their CAD part names; the ring finger uses the middle finger's `M-` meshes.

**Collision shapes** are convex pieces that follow each part's outer surface, skin included: 4 for each fingertip, 8 for the palm, 6 for the tower and one for every other part, each with at most 64 vertices (`<part>_collision*.stl`). On the fingertips' skin around the tip and pad they stay within about 1 mm of the real surface; elsewhere they bridge hollows, by up to about 7 mm on the finger segments and 10 mm on the palm and tower. The URDF and the MJCF use the same pieces; in the MJCF they are in geom group 3, and the visual geoms don't collide. The MJCF excludes contacts between the palm and the tower, and between the palm and each finger's first segment and the thumb's second part, whose pieces meet across a joint; exclude the same pairs when you enable self-collision with the URDF.

### Renaming your files

`v2/name_changes.csv` lists every old name with its new one (empty if removed). `utils/migrate_v2_names.py` applies it to your files, one hand at a time. Without `--write` it only previews; commit your files, then add `--write`:
```bash
python utils/migrate_v2_names.py --source main --side right FILE...
```
For names from the `fb/split-mjcf-scene-positioning` branch, use `--source fb-split-mjcf`. Files with the other hand's or the other source's names are skipped, with exit code 1.

## v2 kinematics and masses

**Joints** follow the v2.1 CAD assembly (November 2026); the left hand is the exact mirror of the right. At 0° the palm is in line with the forearm.

**Masses** were weighed on a v2.1.1 right hand with Dynamixel motors. **Centres of mass and inertias** come from the CAD, with each part at its weighed mass. The URDF and the MJCF have the same masses, centres of mass and inertias.

**Joint damping, friction and armature** are estimates and only in the MJCF. The URDF has none: URDF can't express armature, and damping without armature makes simulators that apply damping explicitly unstable.

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
`utils/make_v2_collisions.py` makes the collision shapes from the right hand's meshes, and `utils/mirror_left_from_right.py` generates the left hand from the right (`--check` verifies it). They need `utils/utils_requirements.txt`. MuJoCo loads STL files of up to 200,000 triangles; the tests check every v2 mesh.

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
