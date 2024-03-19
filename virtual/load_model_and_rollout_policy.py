from stable_baselines3 import TD3, PPO
from utils.other_utils import *
import imageio
import numpy as np

save_dir = "./logs"
total_timesteps=1e6
model = PPO.load(f"{save_dir}/best_model_19.03", verbose=1)
print(f"loaded model: gamma={model.gamma}, n_steps={model.n_steps}")
print("creating env ...")
env = suite.make(
            env_name="Lift",
            robots=["Panda"],
            has_renderer=True,
            render_camera="frontview",
            has_offscreen_renderer=False,
            use_camera_obs=False,  # Not using camera observations
            reward_shaping=True
)
env = GymWrapper(env)
print("finished creating env ...")
model.set_env(env)
obs, info = env.reset()
episode_reward = 0
max_steps_per_episode=int(1e4)
print_freq=100
print("rolling out the policy ...")
with imageio.get_writer('D:/Videos_D/policy_rollout.mp4', fps=20) as video:
    for i in range(max_steps_per_episode):
        if i % print_freq == 0:
            print(f"step: {i}")
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        img = env.env.sim.render(width=800, height=600, camera_name='frontview')  # Adjust as needed
        img = np.flip(img, axis=0)
        video.append_data(img)
        episode_reward += reward
        if terminated or truncated or info.get("is_success", False):
            print("finished episode")
            print("Total Reward:", round(episode_reward,2))
            print("Success?", info.get("is_success", False))
            episode_reward = 0.0
            obs, info = env.reset()
            break