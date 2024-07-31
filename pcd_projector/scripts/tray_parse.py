#!/usr/bin/env python2

ROBOTICA_AVANZADA = False

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from collections import deque
import heapq
import csv
import os
import time

# Constants
PATH_PLANNING_HZ = 2
POINTCLOUD_SCANS_FOR_MAPPING = 2
ODOM_MSGS_FOR_SAVING = 5
MIN_DISTANCE_TO_GOAL = 0.1
DISTANCE_NONAV = 0.8
DECAY_TIME = -1  # Set to -1 for no decay time, otherwise specify decay time in seconds
MAX_POINTS = 10000  # Maximum number of points to keep
FEATHERING = 8  # Feathering size for the costmap gradient for obstacles
PAST_POS_FEATHERING = 2  # Feathering size for the costmap gradient for past positions
RESOLUTION = 0.05  # Grid resolution in meters per cell
WAYPOINT_DISTANCE = 0.4  # Distance between waypoints in meters
UNEXPLORED_COST = 40  # Cost for unexplored areas
DETECTED_POINT_SIZE = 2  # Size of detected points
ROBOT_PAST_POSITION_SIZE = 1  # Size of robot past positions
OBSTACLE_COST = 255  # Cost for obstacles
TRAVERSAL_COST = 10  # Cost for traversed paths
SHOW_IMAGES = False  # Boolean to control if images will be shown on the computer
GOALS_FILE = "/home/alvaro/Desktop/AGV_Data/navigation_goals.csv"  # Path to the CSV file
TRAJECTORIES_FILE = "/home/alvaro/Desktop/AGV_Data/trajectories.csv"

# New constants for modifications
SMOOTHING_FACTOR = 0  # Smoothing factor for path smoothing
DIRECTION_CHANGE_PENALTY = 0  # Penalty for changing direction
HYSTERESIS_LENGTH = 10  # Length of decision history for hysteresis

class PointCloudToImage:
    def __init__(self):
        self.point_cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.point_cloud_callback, queue_size=1, buff_size=2**24)
        self.gripper_pub = rospy.Publisher('/gripper', Bool, queue_size=10)
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        self.trajectory_pub = rospy.Publisher('/next_goal', Point, queue_size=10)
        self.img_map_pub = rospy.Publisher('/img_map', Image, queue_size=10)
        self.img_heatmap_pub = rospy.Publisher('/img_heatmap', Image, queue_size=10)
        self.image_size = (100, 100)
        self.scale_factor = 9  # Adjusted scale factor to fit the points within the image
        self.min_height_threshold = 0.0  # 20 cm
        self.max_height_threshold = 0.1  # 100 cm
        self.frame_rate = 2  # frame rate for updating the image
        self.points = deque(maxlen=MAX_POINTS)  # Use deque to store points with a max length
        self.robot_position = (0.0, 0.0)  # Initialize robot position
        self.feathering_kernel_obstacles = self.create_feathering_kernel(FEATHERING)
        self.feathering_kernel_past_positions = self.create_feathering_kernel(PAST_POS_FEATHERING)
        self.past_positions = deque(maxlen=MAX_POINTS * 2)  # Store past positions of the robot
        self.bridge = CvBridge()
        self.costmap = None  # Initialize costmap as an instance variable
        self.current_goal = None
        self.decision_history = deque(maxlen=HYSTERESIS_LENGTH)  # History for hysteresis
        self.no_more_goals = False
        self.distance_tog = False # Hysteresis de distancia
        self.load_past_positions_from_csv()
        self.cloud_counter = 0
        self.odom_counter = 0
        self.last_path_planning_time = 0

    def read_goals_from_csv(self):
        with open(GOALS_FILE, 'r') as f:
            reader = csv.DictReader(f)
            goals = [row for row in reader if row['Reached'] == '0']
        return goals

    def update_goal_in_csv(self, point_number):
        updated_rows = []
        with open(GOALS_FILE, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['PointNumber'] == str(point_number):
                    row['Reached'] = '1'
                updated_rows.append(row)

        with open(GOALS_FILE, 'w') as f:
            fieldnames = ['PointNumber', 'X', 'Y', 'Reached']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(updated_rows)

    def save_past_positions_to_csv(self):
        with open(TRAJECTORIES_FILE, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(self.robot_position)

    def load_past_positions_from_csv(self):
        if os.path.exists(TRAJECTORIES_FILE):
            with open(TRAJECTORIES_FILE, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    self.past_positions.append((float(row[0]), float(row[1])))

    def point_cloud_callback(self, msg):
        self.cloud_counter += 1

        if self.cloud_counter % POINTCLOUD_SCANS_FOR_MAPPING == 0:
            current_time = rospy.get_time()
            new_points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

            # Downsample the point cloud for faster processing
            if len(new_points) > MAX_POINTS:
                indices = np.random.choice(len(new_points), MAX_POINTS, replace=False)
                new_points = new_points[indices]

            filtered_points = new_points[(new_points[:, 2] >= self.min_height_threshold) & (new_points[:, 2] <= self.max_height_threshold)]

            # Append new points to the deque
            self.points.extend((point[0], point[1], point[2], current_time) for point in filtered_points)

            self.generate_image()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.robot_position = (position.x, position.y)
        # Add the current robot position to the past positions list
        self.past_positions.append((position.x, position.y))
        # Save past positions to CSV file
        if self.odom_counter % ODOM_MSGS_FOR_SAVING == 0:
            self.save_past_positions_to_csv()
        # Check if the robot has reached the goal
        if self.current_goal:
            goal_x, goal_y = -float(self.current_goal['X']), float(self.current_goal['Y'])
            distance_to_goal = np.sqrt((goal_x - position.x)**2 + (goal_y - position.y)**2)

            if distance_to_goal < MIN_DISTANCE_TO_GOAL:
                print("REACHED!")
                self.update_goal_in_csv(self.current_goal['PointNumber'])
                self.current_goal = None  # Reset the current goal
                goals = self.read_goals_from_csv()
                if not goals:
                    self.no_more_goals = True  # Set flag to indicate no more goals

    def create_feathering_kernel(self, size):
        kernel = np.zeros((size * 2 + 1, size * 2 + 1), dtype=np.float32)
        for i in range(-size, size + 1):
            for j in range(-size, size + 1):
                distance = np.sqrt(i**2 + j**2)
                if distance <= size:
                    kernel[size + i, size + j] = (size - distance) / size
        return kernel

    def generate_image(self):
        # Create a black image
        img = np.zeros((self.image_size[0], self.image_size[1], 3), dtype=np.uint8)
        self.costmap = np.full((self.image_size[0], self.image_size[1]), UNEXPLORED_COST, dtype=np.float32)  # Start with a cost of 80 for unexplored areas

        current_time = rospy.get_time()
        new_points = deque(maxlen=MAX_POINTS)  # To store points that haven't decayed

        # Robot's position in the image
        robot_img_x = self.image_size[0] // 2
        robot_img_y = self.image_size[1] // 2

        # Incorporate past positions into the costmap
        past_positions_copy = list(self.past_positions)  # Make a copy of past positions deque
        for pos in past_positions_copy:
            img_x = robot_img_x - int((pos[0] - self.robot_position[0]) * self.scale_factor)
            img_y = robot_img_y + int((pos[1] - self.robot_position[1]) * self.scale_factor)  # Corrected vertical mirroring

            if 0 <= img_x < self.image_size[0] and 0 <= img_y < self.image_size[1]:
                # Apply feathering to past positions
                x_start = max(0, img_x - PAST_POS_FEATHERING)
                y_start = max(0, img_y - PAST_POS_FEATHERING)
                x_end = min(self.image_size[0], img_x + PAST_POS_FEATHERING + 1)
                y_end = min(self.image_size[1], img_y + PAST_POS_FEATHERING + 1)

                kernel_x_start = max(0, PAST_POS_FEATHERING - img_x)
                kernel_y_start = max(0, PAST_POS_FEATHERING - img_y)
                kernel_x_end = kernel_x_start + (x_end - x_start)
                kernel_y_end = kernel_y_start + (y_end - y_start)

                self.costmap[y_start:y_end, x_start:x_end] = np.minimum(
                    self.costmap[y_start:y_end, x_start:x_end],
                    TRAVERSAL_COST + self.feathering_kernel_past_positions[kernel_y_start:kernel_y_end, kernel_x_start:kernel_x_end] * UNEXPLORED_COST
                )

                # Set the point itself to 0
                self.costmap[img_y, img_x] = 0  # Make the point smaller

        for point in self.points:
            x, y, z, timestamp = point

            # Only keep points that haven't decayed
            if DECAY_TIME == -1 or (current_time - timestamp <= DECAY_TIME):
                img_x = robot_img_x - int((x - self.robot_position[0]) * self.scale_factor)
                img_y = robot_img_y + int((y - self.robot_position[1]) * self.scale_factor)  # Corrected vertical mirroring

                if 0 <= img_x < self.image_size[0] and 0 <= img_y < self.image_size[1]:
                    img[img_y, img_x] = (255, 255, 255)  # set the pixel to white
                    new_points.append(point)  # Keep the point

                    # Apply feathering using precomputed kernel for obstacles
                    x_start = max(0, img_x - FEATHERING)
                    y_start = max(0, img_y - FEATHERING)
                    x_end = min(self.image_size[0], img_x + FEATHERING + 1)
                    y_end = min(self.image_size[1], img_y + FEATHERING + 1)

                    kernel_x_start = max(0, FEATHERING - img_x)
                    kernel_y_start = max(0, FEATHERING - img_y)
                    kernel_x_end = kernel_x_start + (x_end - x_start)
                    kernel_y_end = kernel_y_start + (y_end - y_start)

                    self.costmap[y_start:y_end, x_start:x_end] = np.maximum(
                        self.costmap[y_start:y_end, x_start:x_end],
                        self.feathering_kernel_obstacles[kernel_y_start:kernel_y_end, kernel_x_start:kernel_x_end] * OBSTACLE_COST
                    )

        self.points = new_points  # Update points

        # Draw the robot's position as a small square
        robot_x = robot_img_x
        robot_y = robot_img_y

        if 0 <= robot_x < self.image_size[0] and 0 <= robot_y < self.image_size[1]:
            # cv2.rectangle(img, (robot_x - 1, robot_y - 1), (robot_x + 1, robot_y + 1), (0, 255, 0), -1)  # Draw green square
            cv2.circle(img, (robot_x, robot_y), 1, (0, 255, 0), -1)

        current_time = rospy.get_time()
        if current_time - self.last_path_planning_time >= 1.0 / PATH_PLANNING_HZ:
            self.last_path_planning_time = current_time  # Update the last execution time
            tim = rospy.get_time()
            # Set the current goal
            if not self.current_goal:
                goals = self.read_goals_from_csv()
                if goals:
                    self.current_goal = goals[0]  # Get the next unreached goal

            # Generate path using A* algorithm
            start = (robot_x, robot_y)
            if self.current_goal:
                goal = self.global_goal_to_image_coords((float(self.current_goal['X']), float(self.current_goal['Y'])))
                adjusted_goal = self.adjust_goal_to_image_bounds(goal)  # Adjust the goal if necessary
                if adjusted_goal:
                    path = self.a_star(self.costmap, start, adjusted_goal)
                    smoothed_path = self.smooth_path(path)
                    waypoints = self.get_waypoints(smoothed_path)
                    self.draw_path(img, waypoints)
                    self.publish_next_goal(waypoints)
                    # Draw the adjusted goal as a bright magenta circle
                    cv2.circle(img, adjusted_goal, 1, (255, 0, 255), -1)  # Magenta color

            print("T_DELTA = " + str(rospy.get_time() - tim))

        # Normalize costmap to the range 0-255 and convert to 8-bit image
        self.costmap = np.clip(self.costmap, 0, 255).astype(np.uint8)
        costmap_colored = cv2.applyColorMap(self.costmap, cv2.COLORMAP_JET)  # Apply a color map for visualization

        # Convert images to ROS Image messages
        try:
            img_map_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_heatmap_msg = self.bridge.cv2_to_imgmsg(costmap_colored, encoding="bgr8")
            self.img_map_pub.publish(img_map_msg)
            self.img_heatmap_pub.publish(img_heatmap_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

        # Display the images if SHOW_IMAGES is True
        if SHOW_IMAGES:
            cv2.imshow('PointCloud Image', img)
            cv2.imshow('Costmap', costmap_colored)
            cv2.waitKey(1)  # Non-blocking wait

        # Display the updated image with the path if SHOW_IMAGES is True
        if SHOW_IMAGES:
            cv2.imshow('PointCloud Image', img)
            cv2.waitKey(1)  # Non-blocking wait

        goals = self.read_goals_from_csv()
        if goals:
            self.current_goal = goals[0]  # Get the next unreached goal

    def adjust_goal_to_image_bounds(self, goal):
        goal_x, goal_y = goal
        if 0 <= goal_x < self.image_size[0] and 0 <= goal_y < self.image_size[1]:
            return goal

        # If the goal is outside the bounds, find the closest valid point
        closest_goal = None
        min_distance = float('inf')
        for x in range(self.image_size[0]):
            for y in range(self.image_size[1]):
                if self.is_free(x, y):
                    distance = self.heuristic((x, y), goal)
                    if distance < min_distance:
                        min_distance = distance
                        closest_goal = (x, y)

        return closest_goal

    def is_free(self, x, y):
        return 0 <= x < self.image_size[0] and 0 <= y < self.image_size[1] and self.costmap[y, x] < OBSTACLE_COST

    def global_goal_to_image_coords(self, goal):
        goal_x = int(self.image_size[0] / 2 - (-goal[0] - self.robot_position[0]) * self.scale_factor)
        goal_y = int(self.image_size[1] / 2 + (goal[1] - self.robot_position[1]) * self.scale_factor)  # Corrected vertical mirroring
        return (goal_x, goal_y)

    def a_star(self, costmap, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current, costmap.shape):
                cost = costmap[neighbor[1], neighbor[0]]
                if cost == OBSTACLE_COST:  # Skip cells with high cost (walls/obstacles)
                    continue

                direction_penalty = self.direction_change_penalty(current, neighbor, came_from)

                tentative_g_score = g_score[current] + (cost + 1 + direction_penalty)  # Prefer zero-cost paths

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def get_neighbors(self, node, shape):
        neighbors = [
            (node[0] + 1, node[1]),
            (node[0] - 1, node[1]),
            (node[0], node[1] + 1),
            (node[0], node[1] - 1)
        ]
        return [n for n in neighbors if 0 <= n[0] < shape[1] and 0 <= n[1] < shape[0]]

    def heuristic(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def direction_change_penalty(self, current, neighbor, came_from):
        if current in came_from:
            previous = came_from[current]
            prev_dir = (current[0] - previous[0], current[1] - previous[1])
            new_dir = (neighbor[0] - current[0], neighbor[1] - current[1])
            if prev_dir != new_dir:
                return DIRECTION_CHANGE_PENALTY
        return 0

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def smooth_path(self, path):
        if len(path) <= 2:
            return path

        smoothed_path = [path[0]]
        for i in range(1, len(path) - 1):
            prev_point = path[i - 1]
            current_point = path[i]
            next_point = path[i + 1]
            avg_x = (prev_point[0] + next_point[0]) / 2.0
            avg_y = (prev_point[1] + next_point[1]) / 2.0
            smoothed_point = (
                current_point[0] * (1 - SMOOTHING_FACTOR) + avg_x * SMOOTHING_FACTOR,
                current_point[1] * (1 - SMOOTHING_FACTOR) + avg_y * SMOOTHING_FACTOR
            )
            smoothed_path.append(smoothed_point)
        smoothed_path.append(path[-1])
        return smoothed_path

    def get_waypoints(self, path):
        waypoints = []
        distance = 0
        for i in range(1, len(path)):
            prev_point = path[i - 1]
            curr_point = path[i]
            dist = np.sqrt((curr_point[0] - prev_point[0])**2 + (curr_point[1] - prev_point[1])**2) * RESOLUTION
            distance += dist
            if distance >= WAYPOINT_DISTANCE:
                waypoints.append(curr_point)
                distance = 0
        rospy.loginfo("Calculated waypoints: {}".format(waypoints))
        return waypoints

    def draw_path(self, img, waypoints):
        for point in waypoints:
            point = (int(point[0]), int(point[1]))
            cv2.circle(img, point, 1, (0, 0, 255), -1)  # Draw each waypoint as a small red circle

    def publish_next_goal(self, waypoints):
        point_msg = Point()
        goal_x = float(self.current_goal['X'])
        goal_y = float(self.current_goal['Y'])
        distance_to_goal = np.sqrt((-goal_x - self.robot_position[0])**2 + (goal_y - self.robot_position[1])**2)
        # print("Pos x: " + str(self.robot_position[0]) + " --- Pos y: "+ str(self.robot_position[1]))
        # print("Distance: " + str(distance_to_goal))

        if(ROBOTICA_AVANZADA):
            if distance_to_goal < 0.65:
                self.Flag_Aquired()
                return

        if distance_to_goal > 1: # Hysteresis
            self.distance_tog = False

        if self.no_more_goals:
            point_msg.x = self.robot_position[0]
            point_msg.y = self.robot_position[1]
            point_msg.z = 99
            # rospy.loginfo("Publishing no more goals point: ({}, {}, 99)".format(point_msg.x, point_msg.y))
            self.no_more_goals = False  # Reset the flag after publishing
        elif waypoints and not self.distance_tog:
            if self.current_goal:

                if distance_to_goal < DISTANCE_NONAV or self.distance_tog:
                    point_msg.x = -goal_x
                    point_msg.y = goal_y
                    point_msg.z = -1
                    rospy.loginfo(" ---  --- Publishing IF goal directly: ({}, {}, 0)".format(point_msg.x, point_msg.y))
                    self.distance_tog = True
                else:
                    next_goal = waypoints[0]
                    point_msg.x = (next_goal[0] - self.image_size[0] / 2.0) / self.scale_factor + self.robot_position[0]
                    point_msg.y = (self.image_size[1] / 2.0 - next_goal[1]) / self.scale_factor + self.robot_position[1]
                    point_msg.z = 0.0
                    rospy.loginfo("Publishing next goal: ({}, {}, {})".format(point_msg.x, point_msg.y, point_msg.z))
            else:
                rospy.logwarn("No current goal to compare distance with")
        else:
            if distance_to_goal < DISTANCE_NONAV or self.distance_tog:
                point_msg.x = -goal_x
                point_msg.y = goal_y
                point_msg.z = -1
                rospy.loginfo("Publishing goal directly: ({}, {}, 0)".format(point_msg.x, point_msg.y))
                self.distance_tog = True
            else:
                point_msg.x = self.robot_position[0]
                point_msg.y = self.robot_position[1]
                point_msg.z = 99
                rospy.logwarn("No waypoints found to publish")

        self.trajectory_pub.publish(point_msg)

    def Flag_Aquired(self):
        #Parar Robot
        point_msg = Point()
        point_msg.x = self.robot_position[0]
        point_msg.y = self.robot_position[1]
        point_msg.z = 99
        self.trajectory_pub.publish(point_msg)
        rospy.sleep(1)

        #Grippear
        self.gripper_pub.publish(True)
        rospy.loginfo("GRIPPING")
        rospy.sleep(3)

        #Setear nuevo Waypoint a (0,0)
        with open(GOALS_FILE, 'w') as f:
            fieldnames = ['PointNumber', 'X', 'Y', 'Reached']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            points = {'PointNumber': '0', 'X': '0', 'Y': '0', 'Reached': '0'}
            writer.writerow(points)

        self.read_goals_from_csv()
        self.current_goal['X'] = 0
        self.current_goal['Y'] = 0

    def run(self):
        rospy.init_node('point_cloud_to_image', anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    try:
        pc_to_img = PointCloudToImage()
        pc_to_img.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
