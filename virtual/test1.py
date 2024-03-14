import robosuite as suite
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import gym
import numpy as np


class FlattenRobosuiteObservationWrapper:
    def __init__(self, env):
        self.env = env
        # Example observation parts you might want to include (adjust as needed)
        self.obs_parts = ['robot0_proprio-state', 'object-state']

    def reset(self):
        """Resets the environment and returns the initial observation in a flattened format."""
        obs = self.env.reset()
        return self._flatten_observation(obs)

    def step(self, action):
        """Steps the environment using the given action and returns the flattened observation, reward, done, and info."""
        obs, reward, done, info = self.env.step(action)
        return self._flatten_observation(obs), reward, done, info

    def _flatten_observation(self, obs):
        """Helper method to flatten selected parts of the observation dictionary."""
        flat_obs = np.concatenate([obs[part] for part in self.obs_parts if part in obs])
        return flat_obs

    def render(self, mode='human', **kwargs):
        """Render the environment. Passes all arguments to the underlying RoboSuite render method."""
        return self.env.render(mode, **kwargs)

    def close(self):
        """Close the environment."""
        self.env.close()

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s)."""
        return self.env.seed(seed)

class FlattenObservationWrapper(gym.ObservationWrapper):
    def __init__(self, env):
        super(FlattenObservationWrapper, self).__init__(env)
        # Calculate the size of the flat vector. Adjust based on the observations you plan to include.
        flat_size = env.observation_space.spaces['robot0_proprio-state'].shape[0] + \
                    env.observation_space.spaces['object-state'].shape[0]
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(flat_size,), dtype=np.float32)

    def observation(self, obs):
        # Concatenate the observations you're interested in. Adjust the keys as necessary.
        robot_state = obs['robot0_proprio-state']
        object_state = obs['object-state']
        return np.concatenate([robot_state, object_state])

def make_env(env_name):
    def _init():
        env = suite.make(
            env_name=env_name,
            robots="Panda",
            has_renderer=False,
            render_camera="frontview",
            has_offscreen_renderer=True,
            use_camera_obs=False,  # Not using camera observations
        )
        # Wrap the environment to flatten the observations
        env = FlattenObservationWrapper(env)
        return env
    return _init

env_name = "Lift"
num_envs = 1  # Number of parallel environments
print('creating env ...')
# Create vectorized environment without VecTransposeImage
env = make_vec_env(make_env(env_name), n_envs=num_envs)

# Initialize the PPO agent with MlpPolicy
print('creating model ...')
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_lift_tensorboard/")

# Train the agent
total_timesteps = 100000  # Adjust this to train for longer
print("learning ...")
model.learn(total_timesteps=total_timesteps)

# Save the trained model
model.save("ppo_lift")

# Don't forget to close the environment
env.close()
