import os

import gym
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import TD3, PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import BaseCallback
from utils.other_utils import *


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """

    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, "best_model")
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            print(f'step: {self.n_calls}')
            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), "timesteps")
            print(f"x: {x}")
            print(f"y: {y}")
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f"Num timesteps: {self.num_timesteps}")
                    print(
                        f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}"
                    )

                # New best model, you could save the agent here
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    # Example for saving best model
                    if self.verbose > 0:
                        print(f"Saving new best model to {self.save_path}.zip")
                    self.model.save(self.save_path)

        return True


class RewardLoggerCallback(BaseCallback):
    def __init__(self, check_freq, save_path, verbose=1):
        super(RewardLoggerCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_path = save_path
        self.rewards = []

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Assuming the environment is VecEnv, the info is a list of dictionaries
            reward = self.locals['rewards'][0]  # Taking the reward from the first environment
            print(f"reward: {reward}")
            self.rewards.append(reward)
        return True

    def _on_training_end(self) -> None:
        pass
        # Save the rewards to a CSV file at the end of training
        #pd.DataFrame(self.rewards, columns=['reward']).to_csv(self.save_path, index=False)

class RewardLoggerCallback_tboard(BaseCallback):
    def __init__(self, check_freq, save_path, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.save_path = save_path
        self.rewards = []

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Assuming the environment is VecEnv, the info is a list of dictionaries
            x, y = ts2xy(load_results(self.save_path), "timesteps")
            reward = y  # Taking the reward from the first environment
            print(f"reward: {reward}")
            self.rewards.append(reward)
        return True

    def _on_training_end(self) -> None:
        pass
# Create log dir
log_dir = "./logs"
os.makedirs(log_dir, exist_ok=True)

# Create and wrap the environment
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
# Create the callback: check every 1000 steps
callback = SaveOnBestTrainingRewardCallback(check_freq=100, log_dir=log_dir)
#callback=RewardLoggerCallback_tboard(check_freq=1, save_path=log_dir)
# Create RL model
model = PPO('MlpPolicy', mon_env, verbose=1)
# Train the agent
print("learning ...")
#model.learn(total_timesteps=int(5e4), callback=callback, log_interval=100)
model.learn(total_timesteps=int(5e4), log_interval=100)


