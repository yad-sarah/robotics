#before running this script you need to  upload the sketch
#setup_python_write_to_screen.ino
#to the arduino

import serial
import time

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # or '/dev/ttyACM0' on Unix-like systems
BAUD_RATE = 9600

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Wait for the serial connection to initialize
time.sleep(2)

def write_to_screen(text):
    ser.write(text.encode())  # Encode the string to bytes and send it over serial

write_to_screen("Hello, World!")
time.sleep(10)
ser.close()

print("Text sent to Arduino display.")
