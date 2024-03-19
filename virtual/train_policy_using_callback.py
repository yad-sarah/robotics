#train policy and use callback to save best model and display mean reward (over last 100 episodes)

import os
import numpy as np
from stable_baselines3 import TD3, PPO
from stable_baselines3.common.monitor import Monitor
from utils.other_utils import *
from utils.callback_utils import *

log_dir = "./logs"
total_timesteps=1e6
print("creating env ...")
env = suite.make(
            env_name="Lift",
            robots=["Panda"],
            has_renderer=False,
            render_camera="frontview",
            has_offscreen_renderer=False,
            use_camera_obs=False,  # Not using camera observations
            reward_shaping=True
)
env = GymWrapper(env)
env=Monitor(env,log_dir)
print("finished creating env")
#n_actions = env.action_space.n
model = PPO('MlpPolicy', env, verbose=1)
callback_1=SaveOnBestTrainingRewardCallback(check_freq=100,log_dir=log_dir)
os.makedirs(log_dir, exist_ok=True)
print("starting learning ...")
model.learn(total_timesteps=total_timesteps, callback=callback_1 ,log_interval=1)
