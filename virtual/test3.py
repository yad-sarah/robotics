import imageio
import numpy as np
import robosuite as suite
from robosuite.wrappers import GymWrapper

n_steps=100
print("remember that I changed os.environ[MUJOCO_GL] to glfw in robosuite/utils/binding_utils.py" )
print("creating environment ...")
env = suite.make(
    env_name="Lift",  # Example environment
    robots="Panda",   # Choose the robot
    has_renderer=True,  # Enable on-screen rendering
    render_camera="frontview",  # Camera view
    has_offscreen_renderer=False,
    use_camera_obs=False,
)
env=GymWrapper(env)
print("done !")
# Reset the environment
obs = env.reset()
low, high = env.action_spec
for i in range(n_steps):
    print(f"step {i}")
    action = np.random.uniform(low, high)  # Make sure 'low' and 'high' are defined
    obs, reward, done,truncated, info = env.step(action)  # Take a step in the environment
    tmp=7
    if done:
        break
