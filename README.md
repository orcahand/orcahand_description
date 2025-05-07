<img src=".docs/orcahand.png" width="500">

# Orcahand Description

This repository contains the description files for the Orcahand model. Currently, only the MJCF description is available. The 'extended' version contains additional bodies (incl. inertial properties) such as the camera mount, the U2D2 board and fans.

The URDF description will be added soon - for now, use converters such as [mjcf_urdf_simple_converter](https://github.com/Yasu31/mjcf_urdf_simple_converter) or [mjcf2urdf](https://github.com/iory/mjcf2urdf) to convert the MJCF description to URDF.

## Example Usage
1. Clone the repository:
   ```bash
   git clone git@github.com:orcahand/orcahand_description.git
   cd orcahand_description
   ```
2. Install the required dependencies:
   ```bash
   pip install mujoco
   ```
3. Simulate the orcahand in mujoco:
   ```bash
   cd combined
   python3 -m mujoco.viewer --mjcf=scene_combined.xml
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

You can further reduce or mirror meshes using the `mesh_utils.py` script. With the same script, you can also print their number of faces. Some extra dependencies are required:
```bash
pip install trimesh fast_simplification tabulate termcolor
```

We can also recommend the VSCode extension `mtsmfm.vscode-stl-viewer` for quickly visualizing STL meshes.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or support, please contact the maintainers of this repo or the Orcahand team via our website ([https://orcahand.com](https://orcahand.com)).
