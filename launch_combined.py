import mujoco
import mujoco.viewer
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
from utils.placement import place_hand, LEFT_POS, RIGHT_POS, UPRIGHT_QUAT

m = mujoco.MjModel.from_xml_path('scene_combined.xml')
place_hand(m, 'left_root',  pos=LEFT_POS,  quat=UPRIGHT_QUAT)
place_hand(m, 'right_root', pos=RIGHT_POS, quat=UPRIGHT_QUAT)
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)
