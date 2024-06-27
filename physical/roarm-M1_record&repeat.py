from utils.RoArm_control_utils import *

current_positions = get_present_position()
torque_off()
input("Press Enter to return to the recorded position...")
if current_positions:
    return_to_position(current_positions)
