#!/usr/bin/env python3.6
import serial
import time
import os

# Constants
DATA_FILE_PATH = '/tmp/person_track_data.txt'
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
CHECK_INTERVAL = 0.01  # Check file every 0.1 seconds
TIMEOUT = 0.1  # Timeout interval in seconds

# Set up the serial connection
arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=.1)

def read_data():
    with open(DATA_FILE_PATH, 'r') as f:
        return f.read()

def send_to_serial(data):
    arduino.write(data.encode())  # Convert data to bytes and send
    time.sleep(0.05)  # Small delay to ensure data is sent

if __name__ == '__main__':
    last_data = None
    last_modified_time = os.path.getmtime(DATA_FILE_PATH)
    last_data_time = 0
    while True:
        try:
            current_modified_time = os.path.getmtime(DATA_FILE_PATH)
            current_time = time.time()

            if current_modified_time != last_modified_time:
                data = read_data()
                send_to_serial(data)
                last_data = data
                last_modified_time = current_modified_time
                last_data_time = current_time
            elif current_time - last_data_time > TIMEOUT:
                send_to_serial("0000000000")
                last_data_time = current_time

            time.sleep(CHECK_INTERVAL)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
            break
