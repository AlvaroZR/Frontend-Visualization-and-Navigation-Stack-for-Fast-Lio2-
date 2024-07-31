#!/usr/bin/env python

import rospy
import csv
import math
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from scipy.interpolate import CubicSpline
import numpy as np

# Constants
CSV_FILE_PATH = "/home/alvaro/Desktop/AGV_Data/persondata.csv"
NAV_GOALS_FILE_PATH = "/home/alvaro/Desktop/AGV_Data/navigation_goals.csv"
CHECK_INTERVAL = 0.1  # Time in seconds to check the CSV file
CAMERA_OFFSET_X = -0.1  # Camera offset in meters
TARGET_POINT_TOPIC = "/target_point"
ROLLING_AVERAGE_WINDOW = 5  # Maximum length for rolling average
WRITE_TO_CSV = True  # Boolean to specify if data should be written to the CSV file
MIN_POINTS = 4 # For interpolation


# Global variables
odom_position = None
odom_orientation = None
x_points = []
y_points = []
last_published_point = Point()
point_number = 0

print("PERSON PUBLISHER (NO CAMERA)")

def odom_callback(msg):
    global odom_position, odom_orientation
    odom_position = msg.pose.pose.position
    odom_orientation = msg.pose.pose.orientation

def read_csv():
    with open(CSV_FILE_PATH, mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if len(row) == 2:
                x_value = float(row[0]) / 1000.0  # Convert mm to meters
                z_value = float(row[1]) / 1000.0  # Convert mm to meters
                return x_value, z_value
    return None, None

def quaternion_to_euler(x, y, z, w):
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

def publish_target_point(x_value=None, z_value=None):
    global odom_position, odom_orientation, last_published_point
    global x_points, y_points

    if odom_position is None or odom_orientation is None:
        return

    if x_value is not None and z_value is not None and x_value != 0 and z_value != 0:
        # Convert the quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(
            odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w
        )

        # Adjust the point from the camera frame to the robot frame
        camera_x = -z_value  # Camera z is robot x
        camera_y = x_value  # Camera x is negative robot y
        camera_x += CAMERA_OFFSET_X

        # Calculate the point in the map frame (absolute coordinates)
        map_x = -(odom_position.x + camera_x * math.cos(yaw) - camera_y * math.sin(yaw))
        map_y = (odom_position.y + camera_x * math.sin(yaw) + camera_y * math.cos(yaw))

        # Add the points to the list
        x_points.append(map_x)
        y_points.append(map_y)

        # Ensure we have enough points to interpolate
        if len(x_points) >= MIN_POINTS:
            # Interpolate the points
            t = np.arange(len(x_points))
            cs_x = CubicSpline(t, x_points)
            cs_y = CubicSpline(t, y_points)

            # Generate new smoothed points
            t_new = np.linspace(0, len(x_points) - 1, num=len(x_points) * 10)
            x_smooth = cs_x(t_new)
            y_smooth = cs_y(t_new)

            # Use the last smoothed point for publishing
            avg_x = x_smooth[-1]
            avg_y = y_smooth[-1]

            if abs(avg_x - last_published_point.x) > 0.5 or abs(avg_y - last_published_point.y) > 0.5:
                # Update the last published point
                last_published_point.x = avg_x
                last_published_point.y = avg_y
                last_published_point.z = 0  # Assuming the target is on the ground plane

                # Create and publish the point message
                point_msg = Point()
                point_msg.x = avg_x
                point_msg.y = avg_y
                point_msg.z = 0  # Assuming the target is on the ground plane

                target_point_publisher.publish(point_msg)

                # Write to the CSV file if enabled
                if WRITE_TO_CSV and avg_x != 0 and avg_y != 0:
                    global point_number
                    write_to_csv(point_number, avg_x, avg_y)
                    point_number += 1

def write_to_csv(num, x, y):
    with open(NAV_GOALS_FILE_PATH, mode='a') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow([num, x, y, 0])

def write_header():
    with open(NAV_GOALS_FILE_PATH, mode='w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(["PointNumber", "X", "Y", "Reached"])

def main():
    global target_point_publisher

    rospy.init_node('target_point_publisher')

    rospy.Subscriber("/Odometry", Odometry, odom_callback)
    target_point_publisher = rospy.Publisher(TARGET_POINT_TOPIC, Point, queue_size=10)

    rate = rospy.Rate(1 / CHECK_INTERVAL)

    x_pas = None
    z_pas = None
    write_header()

    while not rospy.is_shutdown():
        x_value, z_value = read_csv()
        if x_value != x_pas or z_value != z_pas:
            publish_target_point(x_value, z_value)
            x_pas = x_value
            z_pas = z_value

        rate.sleep()

if __name__ == '__main__':
    main()
