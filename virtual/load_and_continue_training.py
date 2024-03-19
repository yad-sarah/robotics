import os
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import TD3, PPO
import gym
import robosuite as suite
from utils.other_utils import *
from utils.callback_utils import *
from stable_baselines3.common.monitor import Monitor


save_dir = "./logs"
total_timesteps=1e6
loaded_model = PPO.load(f"{save_dir}/best_model_19.03", verbose=1)
print(f"loaded model: gamma={loaded_model.gamma}, n_steps={loaded_model.n_steps}")
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
env=Monitor(env,save_dir)
loaded_model.set_env(env)
callback_1=SaveOnBestTrainingRewardCallback(check_freq=500,log_dir=save_dir)
print("starting learning ...")
loaded_model.learn(total_timesteps=total_timesteps, callback=callback_1 ,log_interval=1)
