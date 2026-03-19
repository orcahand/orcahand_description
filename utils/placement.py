"""
Placement constants and helpers for the ORCA hand MuJoCo models.

The MJCF files (orcahand_left.mjcf / orcahand_right.mjcf) are neutral —
no position or orientation is baked in. Use the constants and helper below
to place the hand(s) in your simulation.

Default upright placement:
  - Rotation: +90° around X  →  Fusion Y-axis maps to MuJoCo Z-up
  - Z offset: 0.058 m        →  lifts ForeArmStructure base to floor level
"""

import mujoco
import numpy as np

# +90° around X: [w, x, y, z]
UPRIGHT_QUAT = [0.7071068, 0.7071068, 0.0, 0.0]

# Z offset so the base of the tower sits on the floor (z=0)
FLOOR_Z_OFFSET = 0.058

# Default placements for standalone and combined scenes
LEFT_POS  = [-0.15, 0.0, FLOOR_Z_OFFSET]
RIGHT_POS = [ 0.15, 0.0, FLOOR_Z_OFFSET]
CENTER_POS = [0.0,  0.0, FLOOR_Z_OFFSET]


def place_hand(model: mujoco.MjModel, body_name: str, pos=CENTER_POS, quat=UPRIGHT_QUAT):
    """
    Set position and orientation of an orcahand root body in a compiled MjModel.

    Args:
        model:      Compiled MjModel (mutable after from_xml_path / mjSpec.compile)
        body_name:  Root body name, e.g. 'left_root' or 'right_root'
        pos:        [x, y, z] position in metres
        quat:       [w, x, y, z] quaternion
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id == -1:
        raise ValueError(f"Body '{body_name}' not found in model")
    model.body_pos[body_id]  = pos
    model.body_quat[body_id] = quat
