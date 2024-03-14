import robosuite as suite
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import gym
import numpy as np

class RobosuiteGymWrapper(gym.Env):
    """A Gym wrapper for RoboSuite environments."""

    def __init__(self, env_name, robots, **kwargs):
        super(RobosuiteGymWrapper, self).__init__()

        # Initialize the RoboSuite environment
        self.env = suite.make(
            env_name=env_name,
            robots=robots,
            **kwargs
        )

        # Define the Gym action space (adjust according to your specific environment)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self.env.action_dim,), dtype=np.float32)

        # Define the Gym observation space (adjust according to your specific environment)
        obs = self.env.reset()
        obs_dim = sum([v.shape[0] for k, v in obs.items() if k.endswith("-state")])
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

    def reset(self):
        """Resets the environment and returns an initial observation."""
        obs = self.env.reset()
        return self._flatten_observation(obs)

    def step(self, action):
        """Steps the environment using the given action."""
        obs, reward, done, info = self.env.step(action)
        return self._flatten_observation(obs), reward, done, info

    def render(self, mode='human', **kwargs):
        """Renders the environment."""
        return self.env.render(mode=mode, **kwargs)

    def close(self):
        """Closes the environment."""
        self.env.close()

    def _flatten_observation(self, obs):
        """Flattens the observation dictionary into a single array."""
        flat_obs = np.concatenate([v for k, v in obs.items() if k.endswith("-state")])
        return flat_obs


