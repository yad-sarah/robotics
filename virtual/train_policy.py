import robosuite as suite
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import numpy as np
from utils.other_utils import *
total_timesteps = 10  #for training

vec_env = make_vec_env(make_env_fn())
print('creating model ...')
model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log="./logs/ppo_lift_tensorboard/")
print("learning ...")
model.learn(total_timesteps=total_timesteps)
model.save("ppo_lift")

# Don't forget to close the environment
#env.close()
