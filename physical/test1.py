from utils.RoArm_control_utils import *
import time

torque_off()
while True:
    pos = get_present_position()
    print(pos)
    time.sleep(1)



