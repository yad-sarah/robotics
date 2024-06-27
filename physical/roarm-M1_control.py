from utils.RoArm_control_utils import *

# Example usage
#torque_on()  # Turn torque on
torch_off()  # Turn torque off
adjust_servo_angle(3, -0.1)  # Increase angle o
#adjust_coord(1, 0.1)  # Increase X coordinate
#emergency_stop()  # Emergency Stop
pos=get_present_position()
print(pos)

