
import requests

BASE_URL = "http://10.0.0.6/"

def send_command(inputT, inputA, inputB):
    url = f"{BASE_URL}cmd?inputT={inputT}&inputA={inputA}&inputB={inputB}"
    try:
        response = requests.get(url, timeout=5)
        print(f"Command sent: inputT={inputT}, inputA={inputA}, inputB={inputB}. Response: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending command: {e}")

def get_present_position():
    url = f"{BASE_URL}readData"
    try:
        response = requests.get(url, timeout=5)
        data = response.json() 
        print("Current Position/Data:", data)
        return data
    except requests.exceptions.RequestException as e:
        print(f"Error fetching present position: {e}")
        return None

def torque_off():
    send_command(2, 0, 0)

def return_to_position(position_data):
    for servo, position in position_data.items():
        if servo.startswith('A'):
            servo_number = int(servo[1:])  
            send_command(1, servo_number, position)  

if __name__ == "__main__":
    current_positions = get_present_position()

    torque_off()

    input("Press Enter to return to the recorded position...")

    if current_positions:
        return_to_position(current_positions)
