#!/usr/bin/env python2

import socket
import time
import subprocess
import serial

def initialize_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("Connected to {} at {} baudrate.".format(port, baudrate))
        time.sleep(1)
        send_message(ser, "@")
        return ser
    except serial.SerialException as e:
        print("Error: {}".format(e))
        quit()
        return None


def send_message(ser, message):
    if ser and ser.isOpen():
        ser.write(message.encode('utf-8'))
        # print("Sent: {}".format(message))
    else:
        print("Serial port not open or not initialized.")
        quit()



port = '/dev/ttyACM5'
baudrate = 115200
ser = initialize_serial(port, baudrate)
time.sleep(2)

def get_connected_network():
    try:
        # Run the command to get the SSID of the connected network
        result = subprocess.check_output(
            ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
            universal_newlines=True
        )
        # Filter the output to get the SSID of the active connection
        for line in result.splitlines():
            if "yes:" in line:
                ssid = line.split(":")[1]
                return ssid

        return "NO NETWORK"  # No active connection found

    except subprocess.CalledProcessError as e:
        print("Error executing command: {}".format(e))
        quit()
        return None

def get_ip_address():
    try:
        # Connect to an external host to determine the IP address
        # This does not actually establish a connection but is used to get the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(('8.8.8.8', 1))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except Exception as e:
        print("Error obtaining IP address: {}".format(e))
        return "0.0.0.0"

while True:
    network = get_connected_network()
    ip = get_ip_address()
    print("{},{}".format(network or "", ip or ""))
    send_message(ser, network + ";" + ip)
    time.sleep(2)
