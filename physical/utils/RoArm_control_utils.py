import requests
from physical.misc_params import *
# Base URL of your toy's server
#BASE_URL = "http://192.168.20.9/"


def send_command(inputT, inputA, inputB):
    """Send a command to the toy's server."""
    url = f"{BASE_URL}cmd?inputT={inputT}&inputA={inputA}&inputB={inputB}"
    try:
        response = requests.get(url, timeout=5)
        print(f"Command sent: inputT={inputT}, inputA={inputA}, inputB={inputB}. Response: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending command: {e}")


# Angle Control Commands
def angle_control_init():
    send_command(1, 0, 0)


def adjust_servo_angle(servo_id, direction):
    """Direction: 1 for increase, 2 for decrease."""
    send_command(1, servo_id, direction)


# Torque Commands
def torque_off():
    send_command(2, 0, 0)


def torque_on():
    send_command(2, 1, 0)


def torch_off():
    send_command(2, 0, 0)


# Coordinate Control Commands
def coord_control_init():
    send_command(3, 0, 0)


def adjust_coord(coord, direction):
    """Coord: 1 to 5 for X, Y, Z, T, G. Direction: 1 for increase, 2 for decrease."""
    send_command(3, coord, direction)


# Record & Replay Commands
def record_step(step_number):
    send_command(5, 1, step_number)


def move_to_step(step_number):
    send_command(5, 2, step_number)


def remove_step(step_number):
    send_command(5, 3, step_number)


def record_delay_time(delay_time):
    send_command(5, 4, delay_time)


def replay_loop_time(loop_time):
    send_command(5, 5, loop_time)


# Servo Configuration Commands
def adjust_servo_config(servo_id, direction):
    """Servo_id should include the servo number and direction as 11 or 12 for -/+ respectively."""
    send_command(6, servo_id, 0)

def get_present_position():
    url = f"{BASE_URL}readData"
    try:
        response = requests.get(url, timeout=5)
        data = response.json()
        #print("Current Position/Data:", data)
        return data
    except requests.exceptions.RequestException as e:
        print(f"Error fetching present position: {e}")
        return None


def return_to_position(position_data):
    for servo, position in position_data.items():
        if servo.startswith('A'):
            servo_number = int(servo[1:])
            send_command(1, servo_number, position)

# Emergency Stop
def emergency_stop():
    try:
        response = requests.get(f"{BASE_URL}stop", timeout=5)
        print(f"Emergency Stop activated. Response: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"Error activating emergency stop: {e}")
