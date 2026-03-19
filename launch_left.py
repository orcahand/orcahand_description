import mujoco
import mujoco.viewer
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
from utils.placement import place_hand, CENTER_POS, UPRIGHT_QUAT

m = mujoco.MjModel.from_xml_path('scene_left.xml')
place_hand(m, 'left_root', pos=CENTER_POS, quat=UPRIGHT_QUAT)
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)
