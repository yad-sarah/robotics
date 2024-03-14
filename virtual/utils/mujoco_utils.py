from PIL import Image as pil_image

def generate_image_mujoco(renderer,data,opts):
    renderer.update_scene(data, scene_option=opts['scene_option'])
    img_arr = renderer.render()
    img = pil_image.fromarray(img_arr, 'RGB')
    return img

def show_image_mujoco(renderer,data,opts):
    img=generate_image_mujoco(renderer,data,opts)
    img.show()

import mujoco

def display_model_properties(model):
    # Extracting basic properties from the model
    num_dofs = model.nq  # Number of degrees of freedom
    num_joints = model.njnt  # Number of joints
    num_actuators = model.nu  # Number of actuators
    num_bodies = model.nbody  # Number of bodies
    num_sensors = model.nsensor  # Number of sensors

    # Display the properties
    print("Model Properties:")
    print(f"Degrees of Freedom: {num_dofs}")
    print(f"Number of Joints: {num_joints}")
    print(f"Number of Actuators: {num_actuators}")
    print(f"Number of Bodies: {num_bodies}")
    print(f"Number of Sensors: {num_sensors}")

