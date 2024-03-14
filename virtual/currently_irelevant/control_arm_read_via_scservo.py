#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SC Servo model on this example : All models using Protocol SC
# This example is tested with a SC Servo(SC15/SC09), and an URT
#

import sys
import os

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()

else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *  # Uses SC Servo SDK library

# Default setting
SCS_ID = 6  # SC Servo ID : 1
BAUDRATE = 1000000  # SC Servo default baudrate : 1000000
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
print(f'SCS_ID : {SCS_ID}')
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = scscl(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print('ku 1')
        print("[ID:%03d] PresPos:%d PresSpd:%d" % (SCS_ID, scs_present_position, scs_present_speed))
        print('print using packetHandler ...')
        print(packetHandler.getTxRxResult(scs_comm_result))
    else:
        print('ku 2')
        print("[ID:%03d] PresPos:%d PresSpd:%d" % (SCS_ID, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print('ku 3')
        print(packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()