#!/usr/bin/env python

import rospy
import csv
import math
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# Constants
CSV_FILE_PATH = "/home/alvaro/Desktop/AGV_Data/flagdata.csv"
NAV_GOALS_FILE_PATH = "/home/alvaro/Desktop/AGV_Data/navigation_goals.csv"
CHECK_INTERVAL = 0.1  # Time in seconds to check the CSV file
CAMERA_OFFSET_X = -0.1  # Camera offset in meters
TARGET_POINT_TOPIC = "/target_point"
ROLLING_AVERAGE_WINDOW = 10  # Maximum length for rolling average
WRITE_TO_CSV = True  # Boolean to specify if data should be written to the CSV file

# Global variables
odom_position = None
odom_orientation = None
x_points = deque(maxlen=ROLLING_AVERAGE_WINDOW)
y_points = deque(maxlen=ROLLING_AVERAGE_WINDOW)
last_published_point = Point()
print("FLAG PUBLISHER (NO CAMERA)")


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

    if x_value is not None and z_value is not None:
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

        # Add the points to the rolling window
        x_points.append(map_x)
        y_points.append(map_y)

        # Calculate the average of the points
        avg_x = sum(x_points) / len(x_points)
        avg_y = sum(y_points) / len(y_points)

        # Update the last published point
        last_published_point.x = avg_x
        last_published_point.y = avg_y
        last_published_point.z = 0  # Assuming the target is on the ground plane
    else:
        # If no new target detected, use the last published point
        avg_x = last_published_point.x
        avg_y = last_published_point.y

    # Create and publish the point message
    point_msg = Point()
    point_msg.x = avg_x
    point_msg.y = avg_y
    point_msg.z = 0  # Assuming the target is on the ground plane

    target_point_publisher.publish(point_msg)

    # Write to the CSV file if enabled
    if WRITE_TO_CSV and avg_x != 0 and avg_y != 0:
        write_to_csv(avg_x, avg_y)

def write_to_csv(x, y):
    with open(NAV_GOALS_FILE_PATH, mode='w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(["PointNumber", "X", "Y", "Reached"])
        csv_writer.writerow([0, x, y, 0])

def main():
    global target_point_publisher

    rospy.init_node('target_point_publisher')

    rospy.Subscriber("/Odometry", Odometry, odom_callback)
    target_point_publisher = rospy.Publisher(TARGET_POINT_TOPIC, Point, queue_size=10)

    rate = rospy.Rate(1 / CHECK_INTERVAL)

    x_pas = None
    z_pas = None
    while not rospy.is_shutdown():
        x_value, z_value = read_csv()
        if x_value != x_pas or z_value != z_pas:
            publish_target_point(x_value, z_value)
            x_pas = x_value
            z_pas = z_value

        rate.sleep()

if __name__ == '__main__':
    main()
