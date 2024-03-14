import os
import robosuite as suite
import numpy as np
import imageio
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
print("done !")
# Reset the environment
obs = env.reset()
low, high = env.action_spec

# Simulate 10 random actions
for i in range(2):
    print(f"step {i}")
    action = np.random.uniform(low, high)
    #action = env.action_spec.sample()  # Sample a random action
    obs, reward, done, info = env.step(action)  # Take a step in the environment
    #env.render()
    img = env.sim.render(width=400, height=300, camera_name='frontview')
    img = np.flip(img, axis=0)
    imageio.imwrite(f'D:/Pictures_D/image_step_{i}.png', img)
    if done:
        break

# Close the environment
env.close()
