from utils.RoArm_control_utils import *
import time
i=0
torque_off()
while True:
    pos = get_present_position()
    print(pos)
    #time.sleep(1)
    i+=1
    print(i)
