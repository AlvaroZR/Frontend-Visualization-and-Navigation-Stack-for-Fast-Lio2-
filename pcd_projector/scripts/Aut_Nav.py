#!/usr/bin/env python3.6

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import serial
import time

current_pose = None
current_goal = None
stop_flag = None
last_goal_time = None
flip_normal = None

MAX_MOTOR_SPEED = 12
MAX_TURNING_SPEED = 12
GOAL_TIMEOUT = 800

def odom_callback(msg):
    global current_pose
    current_pose = msg.pose.pose
    print(f"Received odometry: {current_pose}")

def goal_callback(msg):
    global current_goal
    global last_goal_time
    global stop_flag
    global flip_normal
    current_goal = (msg.x, msg.y)

    if msg.z == -1:
        flip_normal = True
    else:
        flip_normal = False    

    if msg.z > 0:
        stop_flag = True
    else:
        stop_flag = False

    last_goal_time = rospy.get_time() * 1000
    print(f"Received new goal: {current_goal}")

def initialize_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baudrate.")
        return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

def send_message(ser, message):
    if ser and ser.is_open:
        ser.write(message.encode('utf-8'))
        print(f"Sent: {message}")
    else:
        print("Serial port not open or not initialized.")

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def calculate_motor_speeds(current_pose, goal_x, goal_y):
    current_x = current_pose.position.x
    current_y = current_pose.position.y

    error_x = goal_x - current_x
    error_y = goal_y - current_y
    distance_to_goal = math.sqrt(error_x**2 + error_y**2)

    angle_to_goal = math.atan2(error_y, error_x)

    _, _, yaw = euler_from_quaternion([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ])

    # Correcting the yaw to account for the robot's orientation being inverted
    if(flip_normal):
        yaw += math.pi

    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

    error_angle = angle_to_goal - yaw
    error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

    print(f"Distance to goal: {distance_to_goal}, Angle to goal: {angle_to_goal}, Yaw: {yaw}, Error angle: {error_angle}")

    linear_gain = 10
    angular_dead_zone = math.radians(10)  # Increase dead zone to 10 degrees

    if abs(error_angle) > angular_dead_zone:
        # Only turn if the error angle is larger than the dead zone
        linear_velocity = 0
        angular_velocity = MAX_TURNING_SPEED if error_angle < 0 else -MAX_TURNING_SPEED
    else:
        # Move forward if within the dead zone
        linear_velocity = linear_gain * distance_to_goal
        angular_velocity = 0

    # Ensure linear velocity is set correctly when aligned
    print(f"Linear velocity before scaling: {linear_velocity}")

    # Scale linear velocity proportionally to distance to goal
    linear_velocity = min(MAX_MOTOR_SPEED, max(-MAX_MOTOR_SPEED, linear_velocity))

    # Ensure linear velocity is scaled correctly
    print(f"Scaled linear velocity: {linear_velocity}")
    left_motor_speed = linear_velocity - angular_velocity
    right_motor_speed = linear_velocity + angular_velocity

    # Ensure minimum motor speed for torque
    if linear_velocity != 0:
        min_motor_speed = 8
        if abs(left_motor_speed) > 0 and abs(left_motor_speed) < min_motor_speed:
            left_motor_speed = min_motor_speed if left_motor_speed > 0 else -min_motor_speed
        if abs(right_motor_speed) > 0 and abs(right_motor_speed) < min_motor_speed:
            right_motor_speed = min_motor_speed if right_motor_speed > 0 else -min_motor_speed

    left_motor_speed = round(left_motor_speed)
    right_motor_speed = round(right_motor_speed)

    # Add debug information to check motor speeds
    print(f"Left motor speed: {left_motor_speed}, Right motor speed: {right_motor_speed}")
    if stop_flag == True:
        left_motor_speed = 0
        right_motor_speed = 0

    return left_motor_speed, right_motor_speed

def main():
    rospy.init_node('navigation_node', anonymous=True)
    rospy.Subscriber('/Odometry', Odometry, odom_callback)
    rospy.Subscriber('/next_goal', Point, goal_callback)
    print("ROS node initialized and subscribed to /Odometry and /next_goal")

    port = '/dev/ttyACM0'
    baudrate = 115200
    ser = initialize_serial(port, baudrate)
    time.sleep(2)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        global current_goal
        if current_pose is not None and current_goal is not None:
            goal_x, goal_y = current_goal

            current_time = rospy.get_time() * 1000
            if last_goal_time is not None and (current_time - last_goal_time) > GOAL_TIMEOUT:
                # Timeout exceeded, set motor speeds to zero
                left_motor, right_motor = 0, 0
                print("Goal callback timeout exceeded. Stopping the robot.")
            else:
                left_motor, right_motor = calculate_motor_speeds(current_pose, goal_x, goal_y)
                                                                 
            # left_motor, right_motor = calculate_motor_speeds(current_pose, goal_x, goal_y)
            print(f"PAx: {current_pose.position.x}, PAy: {current_pose.position.y}")
            print(f"PGx: {goal_x}, PGy: {goal_y}")

            output = format_output(left_motor, right_motor)
            send_message(ser, output)

            if goal_reached(current_pose, goal_x, goal_y):
                output = format_output(0, 0)
                send_message(ser, output)
                rospy.loginfo("Goal reached: ({}, {})".format(goal_x, goal_y))
                current_goal = None  # Reset the goal after it is reached

        rate.sleep()

def format_output(left_motor, right_motor):
    left_direction = 1 if left_motor >= 0 else 0
    right_direction = 1 if right_motor >= 0 else 0

    left_speed = abs(left_motor)
    right_speed = abs(right_motor)

    left_brake = 1 if left_speed > 0 else 0
    right_brake = 1 if right_speed > 0 else 0

    output = "{:01d}{:03d}{:01d}{:01d}{:03d}{:01d}".format(left_direction, left_speed, left_brake, right_direction, right_speed, right_brake)
    return output

def goal_reached(current_pose, goal_x, goal_y):
    current_x = current_pose.position.x
    current_y = current_pose.position.y
    distance_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

    goal_threshold = 0.05  # meters
    return distance_to_goal < goal_threshold

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
