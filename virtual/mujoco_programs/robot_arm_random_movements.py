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
duration = 10.0
ctrl_rate = 2
ctrl_std = 0.05
total_rot = 60
blend_std = .8
# Make model and data
model = mujoco.MjModel.from_xml_path("mujoco_models/trossen_vx300s/scene.xml")
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
show_image_mujoco(renderer,data,render_opts)
# Set the desired control point.
# Sample actuator noise and smooth it.
nsteps = int(np.ceil(duration / model.opt.timestep))
perturb = np.random.randn(nsteps, model.nu)
width = int(nsteps * ctrl_rate / duration)
kernel = np.exp(-0.5 * np.linspace(-3, 3, width) ** 2)
kernel /= np.linalg.norm(kernel)
for i in range(model.nu):
  perturb[:, i] = np.convolve(perturb[:, i], kernel, mode="same")
if model.nkey > 0:
  mujoco.mj_resetDataKeyframe(model, data, 0)
  ctrl0 = data.ctrl.copy()
else:
  mujoco.mj_resetData(model, data)
  ctrl0 = np.mean(model.actuator_ctrlrange, axis=1)

frames = []
print ('simulating the model ...')
for i in tqdm(range(nsteps)):
  data.ctrl[:] = ctrl0 + ctrl_std * perturb[i]
  mujoco.mj_step(model, data)
  if len(frames) < data.time * fps:
    renderer.update_scene(data, camera=camera_1)
    frame = renderer.render().copy().astype(np.float32)
    frame = frame.astype(np.uint8)
    frames.append(frame)
video_filename="D:/Videos_D/mujoco_robot_arm_3.mp4"
print('generating video from images ...')
create_mp4_from_images(frames,video_filename,fps)
renderer.close()
