# using mujoco 3.2.1
# simulate movement of object with 1 joint (DoF=1)

import mujoco
import mediapy as media
import time
from PIL import Image as pil_image
from utils.mujoco_utils import *

# Make model and data
model = mujoco.MjModel.from_xml_path("mujoco_models/box_sphere.xml")
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)
print(f'DoF: {model.nq}')
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
render_opts={
'scene_option':scene_option
}

mujoco.mj_forward(model, data)
show_image_mujoco(renderer,data,render_opts)
for i in range(100):
    mujoco.mj_step(model, data)
#renderer.update_scene(data)
show_image_mujoco(renderer,data,render_opts)
renderer.close()
