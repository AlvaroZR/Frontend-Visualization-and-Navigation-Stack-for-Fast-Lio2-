#!/usr/bin/env python3.6
import serial
import time
import pygame

# Define maximum motor speed constants (0 to 100 scale)
MAX_MOTOR_SPEED = 30  # percentage

# Initialize serial connection
def initialize_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baudrate.")
        return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

# Format the output string
def format_output(left_motor, right_motor, boost):
    left_direction = 1 if left_motor >= 0 else 0
    right_direction = 1 if right_motor >= 0 else 0

    if boost:
        left_speed = abs(left_motor) + 5
        right_speed = abs(right_motor) + 5
    else:
        left_speed = abs(left_motor)
        right_speed = abs(right_motor)

    left_brake = 1 if left_speed > 0 else 0
    right_brake = 1 if right_speed > 0 else 0

    output = "{:01d}{:03d}{:01d}{:01d}{:03d}{:01d}".format(left_direction, left_speed, left_brake, right_direction, right_speed, right_brake)
    return output

# Send message to the robot
def send_message(ser, message):
    if ser and ser.is_open:
        ser.write(message.encode('utf-8'))
        print(f"Sent: {message}")
    else:
        print("Serial port not open or not initialized.")

# Initialize the PS4 controller
def initialize_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Initialized Joystick: {joystick.get_name()}")
        return joystick
    else:
        print("No joystick found.")
        return None

# Control loop using PS4 controller
def control_robot(ser, joystick, ser_grip):
    current_speed = 10
    dead_zone = 0.1  # Define a dead zone for joystick axes
    last_speed_update_time = time.time()

    running = True
    while running:
        # Pump and handle all events
        pygame.event.pump()
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")
                if joystick.get_button(10):  # PS button
                    running = False
                    break

        if joystick.get_button(0):  # X button
            send_message(ser_grip,'C')

        if joystick.get_button(1):  # O button
            send_message(ser_grip,'O')

        if joystick.get_button(2):
            pygame.mixer.music.load("/home/alvaro/Downloads/venezuela.mp3")
            pygame.mixer.music.play()

        if joystick.get_button(3):
            pygame.mixer.music.load("/home/alvaro/Downloads/perrito.mp3")
            pygame.mixer.music.play()

        if joystick.get_button(4):
            pygame.mixer.music.load("/home/alvaro/Downloads/horn.mp3")
            pygame.mixer.music.play()


        # Check D-pad inputs for speed control
        dpad_horizontal = joystick.get_hat(0)[0]  # D-pad horizontal axis
        current_time = time.time()
        if dpad_horizontal == -1 and current_time - last_speed_update_time >= 0.4:  # D-pad left
            current_speed = max(0, current_speed - 1)
            last_speed_update_time = current_time
            print(f"Decreasing speed: {current_speed}")
        elif dpad_horizontal == 1 and current_time - last_speed_update_time >= 0.4:  # D-pad right
            current_speed = min(MAX_MOTOR_SPEED, current_speed + 1)
            last_speed_update_time = current_time
            print(f"Increasing speed: {current_speed}")

        # Use the left stick (axis 1) and right stick (axis 4) for motor control
        left_axis = -joystick.get_axis(4)
        right_axis = -joystick.get_axis(1)


        # Apply dead zone
        left_motor = int(left_axis * current_speed) if abs(left_axis) > dead_zone else 0
        right_motor = int(right_axis * current_speed) if abs(right_axis) > dead_zone else 0

        print(f"Left Motor: {left_motor}, Right Motor: {right_motor}")

        if joystick.get_button(6) and joystick.get_button(7):
            output = format_output(left_motor, right_motor, True)
        else:
            output = format_output(left_motor, right_motor, False)

        send_message(ser, output)

        # Delay to avoid flooding the console with too many messages
        time.sleep(0.05)

if __name__ == '__main__':
    port = '/dev/ttyACM0'  # Replace with your serial port
    port_gripper = '/dev/ttyACM1'  # Replace with your serial port

    baudrate = 115200  # Replace with your baudrate

    ser = initialize_serial(port, baudrate)
    ser_grip = initialize_serial(port_gripper, baudrate)

    time.sleep(2)  # Give some time for the serial connection to establish

    joystick = initialize_controller()

    if ser and joystick:
        control_robot(ser, joystick,ser_grip)

    pygame.quit()
