# using mujoco 3.2.1
# simulate movement of object with 1 joint (DoF=1)

import mujoco
import mediapy as media
import time
from PIL import Image as pil_image
from utils.mujoco_utils import *
from utils.image_utils import *

box_sphere = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -90">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

framerate = 10  # (Hz)

# Make model and data
model = mujoco.MjModel.from_xml_string(box_sphere)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
render_opts={
'scene_option':scene_option
}

mujoco.mj_forward(model, data)
frames=[]
n_steps_within_mj_step=10
for i in range(100):
    print(f"step {i}")
    mujoco.mj_step(model, data,nstep=n_steps_within_mj_step)
    renderer.update_scene(data, scene_option=render_opts['scene_option'])
    img_arr = renderer.render()
    frames.append(img_arr)

#renderer.update_scene(data)
#media.show_video(frames, fps=framerate)
video_filename="D:/Videos_D/mujoco_demo_1.mp4"
print('generating video from images ...')
create_mp4_from_images(frames,video_filename,framerate)

renderer.close()
