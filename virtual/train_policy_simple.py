#train agent using Monitor() wrapper to log agent's performanc to log_dir

import os
import numpy as np
from stable_baselines3 import TD3, PPO
from stable_baselines3.common.monitor import Monitor
from utils.other_utils import *

log_dir = "./logs"
total_timesteps=int(1e5)
print("creating env ...")
env = suite.make(
            env_name="Lift",
            robots=["Panda"],
            has_renderer=False,
            render_camera="frontview",
            has_offscreen_renderer=False,
            use_camera_obs=False,  # Not using camera observations
        )
env = GymWrapper(env)
mon_env=Monitor(env,log_dir)
print("finished creating env")
#n_actions = env.action_space.n
model = TD3('MlpPolicy', mon_env, verbose=1)
os.makedirs(log_dir, exist_ok=True)
print("starting learning ...")
model.learn(total_timesteps=total_timesteps, log_interval=1)
