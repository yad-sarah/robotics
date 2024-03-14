# using mujoco 3.2.1
# simulate movement of object with 1 joint (DoF=1)

from dataclasses import dataclass
import numpy as np
import mediapy as media
from pathlib import Path
import enum
from tqdm import tqdm
import mujoco
from PIL import Image as pil_image
from utils.mujoco_utils import *
from utils.image_utils import *

custom_camera=True
visualize_joints = False # enable joint visualization
res_fact=1.2
res = (int(res_fact*480),int(res_fact*640))
#res=(720, 1280)
fps = 10
duration = 20.0
#ctrl_rate = 2
#ctrl_std = 0.05
#total_rot = 60
#blend_std = .8
# Make model and data
model = mujoco.MjModel.from_xml_path("mujoco_models/trossen_vx300s/scene.xml")
display_model_properties(model)
model.vis.global_.offheight = res[0]
model.vis.global_.offwidth = res[1]
#sim = mujoco.MjSim(model)
data = mujoco.MjData(model)

renderer = mujoco.Renderer(model,height=res[0],width=res[1])
if custom_camera:
    camera_1 = mujoco.MjvCamera()
    camera_1.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera_1.lookat = [0.0, 0.0, 0.0]  # x position
    camera_1.distance = 1.5
    camera_1.azimuth = 45.0
    camera_1.elevation = -30
else:
    camera_1 = -1


if visualize_joints:
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
else:
    scene_option = None
render_opts={
'scene_option':scene_option,
'camera':camera_1  # make sure that there is a camera in the xml file with this name
}
mujoco.mj_forward(model, data)
#show_image_mujoco(renderer,data,render_opts)
# Set the desired control point.
# Sample actuator noise and smooth it.
nsteps = int(np.ceil(duration / model.opt.timestep))
#reset model to last keyframe or default
if model.nkey > 0:
  mujoco.mj_resetDataKeyframe(model, data, 0)
  ctrl0 = data.ctrl.copy()
else:
  mujoco.mj_resetData(model, data)
  ctrl0 = np.mean(model.actuator_ctrlrange, axis=1)

print(f"value of each actuator")
for i in range(model.nu):
    print(f"actuator {i}: {ctrl0[i]}")

print(f"value range for each actuator")
for i in range(model.nu):
    print(f"actuator {i}: {model.actuator_ctrlrange[i]}")
frames = []
print ('simulating the model ...')
#change of value for each actuator
d_act=[1.0, 1.0, -1.0, 1.0,1.0,1.0,0.02]
steps_per_actuator = int(nsteps / model.nu)
steps_per_direction=int(steps_per_actuator/2)
d_act_per_step=[x/steps_per_actuator for x in d_act]
for i in tqdm(range(model.nu)):
    print(f"changing state of actuator {i}")
    for j in tqdm(range(int(nsteps/model.nu))):
        if j< steps_per_direction:
            data.ctrl[i] += d_act_per_step[i]
        else:
            data.ctrl[i] -= d_act_per_step[i]
      #print(f'ctrl: {data.ctrl}')
        mujoco.mj_step(model, data)
        if len(frames) < data.time * fps:
            renderer.update_scene(data, camera=camera_1)
            frame = renderer.render()
            frames.append(frame)
video_filename="D:/Videos_D/mujoco_robot_arm_12.mp4"
print('generating video from images ...')
create_mp4_from_images(frames,video_filename,fps)
renderer.close()
