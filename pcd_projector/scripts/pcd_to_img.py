#!/usr/bin/env python3.6

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from multiprocessing import Pool, cpu_count
from numba import jit
import cv2
from flask import Flask, send_file, jsonify, make_response, send_from_directory, request
from flask_cors import CORS
import rospy
from nav_msgs.msg import Odometry
import subprocess
import signal
import os
import sys
import json
from threading import Thread
import psutil
import time
import csv

app = Flask(__name__, static_folder='static', static_url_path='')
CORS(app, resources={r"/*": {"origins": "*"}})
NAV_GOALS_FILE = "/home/alvaro/Desktop/AGV_Data/navigation_goals.csv"
save_path = "/home/alvaro/Desktop/AGV_Data/mapa_global.png"
data_file = "/home/alvaro/Desktop/AGV_Data/data.json"
img_map_path = '/tmp/img_map.png'
img_heatmap_path = '/tmp/img_heatmap.png'
img_camera_path = '/tmp/img_camera.png'
process_file = '/tmp/processes.json'

img_map = None
img_heatmap = None
process = None
processes = []

def load_processes():
    global processes
    if os.path.exists(process_file):
        with open(process_file, 'r') as f:
            processes = json.load(f)

def save_processes():
    with open(process_file, 'w') as f:
        json.dump(processes, f)

def is_localtunnel_running(subdomain):
    """Check if a localtunnel instance with the specified subdomain is already running."""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        cmdline = proc.info['cmdline']
        if cmdline and 'lt' in cmdline and subdomain in cmdline:
            return True
    return False

def start_localtunnel():
    print("Starting Localtunnel...")

roscore_process = None
def start_roscore():
    global roscore_process
    if roscore_process is None:
        roscore_process = subprocess.Popen(['roscore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def start_cv_bridge_handler():
    subprocess.Popen(['python2.7', '/home/alvaro/pcd_ws/src/pcd_projector/scripts/cv_bridge_handler.py'])

def signal_handler(sig, frame):
    """Handle termination signals to gracefully shutdown the server."""
    if process and process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    if roscore_process:
        roscore_process.terminate()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def create_unavailable_image(text):
    img = np.zeros((500, 500, 3), dtype=np.uint8)
    cv2.putText(img, text, (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    return img

@app.route('/img_map')
def serve_img_map():
    if os.path.exists(img_map_path):
        return send_file(img_map_path, mimetype='image/png')
    else:
        unavailable_img = create_unavailable_image("Map not Available!")
        _, img_encoded = cv2.imencode('.png', unavailable_img)
        response = make_response(img_encoded.tobytes())
        response.headers['Content-Type'] = 'image/png'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/img_heatmap')
def serve_img_heatmap():
    if os.path.exists(img_heatmap_path):
        return send_file(img_heatmap_path, mimetype='image/png')
    else:
        unavailable_img = create_unavailable_image("Heatmap not Available!")
        _, img_encoded = cv2.imencode('.png', unavailable_img)
        response = make_response(img_encoded.tobytes())
        response.headers['Content-Type'] = 'image/png'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/img_camera')
def serve_img_camera():
    if os.path.exists(img_camera_path):
        return send_file(img_camera_path, mimetype='image/png')
    else:
        unavailable_img = create_unavailable_image("Camera not Available!")
        _, img_encoded = cv2.imencode('.png', unavailable_img)
        response = make_response(img_encoded.tobytes())
        response.headers['Content-Type'] = 'image/png'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response


@app.route('/reprocess_pointcloud', methods=['POST'])
def reprocess_pointcloud():
    try:
        main()
        return jsonify({'message': 'Point cloud reprocessed successfully'}), 200
    except Exception as e:
        print(f"Error reprocessing point cloud: {str(e)}", file=sys.stderr)
        return jsonify({'error': str(e)}), 500

@app.route('/')
def serve_html():
    return send_from_directory('static', 'index.html')

@app.route('/image_ready')
def image_ready():
    if os.path.exists(save_path):
        return jsonify({"ready": True}), 200
    return jsonify({"ready": False}), 200

def kill_processes_by_env(tag):
    try:
        for proc in psutil.process_iter(attrs=['pid', 'environ']):
            try:
                env = proc.environ()
                if 'COMMAND_TAG' in env and env['COMMAND_TAG'] == tag:
                    pgid = os.getpgid(proc.info['pid'])
                    print(f"Sending SIGINT to process group {pgid} (process with PID {proc.info['pid']})", file=sys.stderr)
                    os.killpg(pgid, signal.SIGINT)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return True
    except Exception as e:
        print(f"Error stopping processes by environment variable: {str(e)}", file=sys.stderr)
        return False

@app.route('/run_command', methods=['POST'])
def run_command():
    try:
        command = request.json.get('command')
        tag = request.json.get('tag')
        name = request.json.get('name')  # Get the name from the request
        if command and tag and name:
            full_command = f'gnome-terminal -- bash -c "export COMMAND_TAG={tag}; {command}; if [ $? -eq 0 ]; then echo \'Process completed.\'; sleep 1; fi; exit"'
            process = subprocess.Popen(
                full_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setpgrp,
                universal_newlines=True
            )

            print(f"Started gnome-terminal with PID {process.pid} running command {command} with tag {tag} and name {name}", file=sys.stderr)
            processes.append({"pid": process.pid, "tag": tag, "command": command, "name": name})
            save_processes()

            return jsonify({'output': 'Command started: ' + name, 'pid': process.pid, 'tag': tag, 'name': name})
        return jsonify({'error': 'No command, tag, or name provided'}), 400
    except Exception as e:
        print(f"Error executing command: {str(e)}", file=sys.stderr)
        return jsonify({'error': str(e)}), 500

@app.route('/stop_command', methods=['POST'])
def stop_command():
    try:
        tag = request.json.get('tag')
        if tag:
            success = kill_processes_by_env(tag)
            if success:
                processes[:] = [p for p in processes if p['tag'] != tag]
                save_processes()
                return jsonify({'message': f'Process with tag {tag} interrupted'})
            return jsonify({'error': f'Failed to interrupt processes with tag {tag}'}), 400
        return jsonify({'error': 'No tag provided'}), 400
    except Exception as e:
        print(f"Error stopping command: {str(e)}", file=sys.stderr)
        return jsonify({'error': str(e)}), 500


@app.route('/update_goals', methods=['POST'])
def update_goals():
    goals = request.json
    existing_goals = {}

    # Read the existing goals from the file and store them in a dictionary
    if os.path.exists(NAV_GOALS_FILE):
        with open(NAV_GOALS_FILE, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                existing_goals[(float(row['X']), float(row['Y']))] = row['Reached']

    # Write the updated goals to the file, preserving the Reached status for existing goals
    with open(NAV_GOALS_FILE, 'w') as f:
        f.write("PointNumber,X,Y,Reached\n")
        for i, goal in enumerate(goals):
            reached_status = existing_goals.get((goal['x'], goal['y']), '0')
            f.write(f"{i},{goal['x']},{goal['y']},{reached_status}\n")

    return jsonify({"status": "success", "message": "Navigation goals updated"}), 200


@app.route('/get_goals', methods=['GET'])
def get_goals():
    if not os.path.exists(NAV_GOALS_FILE):
        return jsonify([])  # Return empty list if file does not exist

    goals = []
    with open(NAV_GOALS_FILE, 'r') as f:
        next(f)  # Skip header
        for line in f:
            point_number, x, y, reached = line.strip().split(',')
            goals.append({"x": float(x), "y": float(y), "r": int(reached)})

    return jsonify(goals)


@app.route('/system_usage', methods=['GET'])
def system_usage():
    cpu_usage = psutil.cpu_percent(interval=1)
    memory_usage = psutil.virtual_memory().percent
    swap_usage = psutil.swap_memory().percent
    return jsonify({
        'cpu': cpu_usage,
        'memory': memory_usage,
        'swap': swap_usage
    })

@app.route('/get_processes', methods=['GET'])
def get_processes():
    return jsonify(processes)

x_ratio = 0
y_ratio = 0
bounding_box = {}
robot_position = {"x": 0.0, "y": 0.0, "z": 0.0, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}

def save_data():
    data = {
        "x_ratio": x_ratio,
        "y_ratio": y_ratio,
        "bounding_box": bounding_box,
        "robot_position": robot_position
    }
    with open(data_file, 'w') as f:
        json.dump(data, f)

def delete_image(save_path):
    try:
        os.remove(save_path)
        print(f"File {save_path} deleted successfully.")
    except FileNotFoundError:
        print(f"File {save_path} not found.")
    except PermissionError:
        print(f"Permission denied: unable to delete {save_path}.")
    except Exception as e:
        print(f"Error occurred while deleting file {save_path}: {e}")

def load_data():
    global x_ratio, y_ratio, bounding_box, robot_position
    if os.path.exists(data_file):
        with open(data_file, 'r') as f:
            data = json.load(f)
            x_ratio = data.get("x_ratio", 0)
            y_ratio = data.get("y_ratio", 0)
            bounding_box = data.get("bounding_box", {})
            robot_position = data.get("robot_position", {"x": 0.0, "y": 0.0, "z": 0.0, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}})

def color_map(z, min_z, max_z):
    if max_z == min_z:
        normalized_z = 0
    else:
        normalized_z = (z - min_z) / (max_z - min_z)
    cmap = plt.get_cmap('viridis')
    color = cmap(normalized_z)
    contrast_factor = 1
    enhanced_color = np.array(color[:3]) ** contrast_factor
    return enhanced_color

def prepare_color_lookup(min_z, max_z):
    return np.array([color_map(z, min_z, max_z) for z in np.linspace(min_z, max_z, 256)])

def remove_islands(image):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8, ltype=cv2.CV_32S)
    largest_component_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    largest_component = np.zeros_like(image, dtype=np.uint8)
    largest_component[labels == largest_component_label] = 255
    return largest_component

def enhance_image(image):
    gray_image = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY)
    _, binary_image = cv2.threshold(gray_image, 10, 255, cv2.THRESH_BINARY)
    connected_image = remove_islands(binary_image)
    final_image = cv2.bitwise_and(image, image, mask=connected_image)
    # final_image = image

    return final_image

@jit(nopython=True)
def process_points(points_chunk, p_min, p_max, scale, z_threshold, color_lookup, ground, x_min, y_min, resolution, point_size, origin_pos, square_size, ground_threshold):
    img_width, img_height = scale
    image = np.zeros((img_height, img_width, 3))
    x_ratio = (img_width - 1) / (p_max[0] - p_min[0])
    y_ratio = (img_height - 1) / (p_max[1] - p_min[1])

    for x, y, z in points_chunk:
        ix = int((x - p_min[0]) * x_ratio)
        iy = int((y - p_min[1]) * y_ratio)
        ground_ix = int((x - x_min) // resolution)
        ground_iy = int((y - y_min) // resolution)
        ground_z = ground[ground_iy, ground_ix]

        if z - ground_z > ground_threshold:
            continue

        relative_z = (z - p_min[2]) / (p_max[2] - p_min[2])
        color_index = int(relative_z * 255)
        color = color_lookup[color_index]

        for dx in range(-point_size // 2, point_size // 2 + 1):
            for dy in range(-point_size // 2, point_size // 2 + 1):
                new_ix, new_iy = ix + dx, iy + dy
                if 0 <= new_ix < img_width and 0 <= new_iy < img_height:
                    image[new_iy, new_ix] = np.maximum(image[new_iy, new_ix], color)

    return image

def remove_outliers(points, z_threshold=2.0):
    mean_z = np.mean(points[:, 2])
    std_z = np.std(points[:, 2])
    filtered_points = points[np.abs(points[:, 2] - mean_z) / std_z < z_threshold]
    return filtered_points

@app.route('/image')
def serve_image():
    image_path = save_path
    response = make_response(send_file(image_path, mimetype='image/png'))
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/robotthumbail')
def serve_thumbail():
    image_path = "/home/alvaro/Desktop/AGV_Data/RobotThumbail.png"
    response = make_response(send_file(image_path, mimetype='image/png'))
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/data')
def serve_data():
    data = {
        "x_ratio": x_ratio,
        "y_ratio": y_ratio,
        "bounding_box": bounding_box
    }
    return jsonify(data)

@app.route('/robot_position')
def get_robot_position():
    return jsonify(robot_position)

def odometry_callback(msg):
    global robot_position
    robot_position["x"] = msg.pose.pose.position.x
    robot_position["y"] = msg.pose.pose.position.y
    robot_position["z"] = msg.pose.pose.position.z
    robot_position["orientation"] = {
        "x": msg.pose.pose.orientation.x,
        "y": msg.pose.pose.orientation.y,
        "z": msg.pose.pose.orientation.z,
        "w": msg.pose.pose.orientation.w
    }

def main():
    delete_image(save_path)
    print("Loading point cloud...")
    pcd = o3d.io.read_point_cloud("/home/alvaro/FastLio/src/FAST_LIO/PCD/scans.pcd")
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    points = np.asarray(pcd.points)
    print("Removing outliers...")
    points = remove_outliers(points)

    p_min, p_max = points.min(axis=0), points.max(axis=0)
    color_lookup = prepare_color_lookup(p_min[2], p_max[2])

    aspect_ratio = (p_max[1] - p_min[1]) / (p_max[0] - p_min[0])
    img_width = 4096
    img_height = int(img_width * aspect_ratio)
    scale = (img_width, img_height)

    global bounding_box
    bounding_box = {
        "x_min": p_min[0],
        "x_max": p_max[0],
        "y_min": p_min[1],
        "y_max": p_max[1]
    }

    ground = np.zeros((img_height, img_width))
    x_min = p_min[0]
    y_min = p_min[1]
    resolution = 0.5

    num_cpus = 1 #max(1, cpu_count() // 2)
    points_split = np.array_split(points, num_cpus)
    point_size = 10

    square_size = 15
    global x_ratio
    global y_ratio
    x_ratio = (img_width - 1) / (p_max[0] - p_min[0])
    y_ratio = (img_height - 1) / (p_max[1] - p_min[1])

    ground_threshold = 1.5
    args = [(chunk, p_min, p_max, scale, p_max[2] - 2, color_lookup, ground, x_min, y_min, resolution, point_size, (0, 0), square_size, ground_threshold)
            for chunk in points_split]

    print("Processing Points ...")
    with Pool() as pool:
        results = pool.starmap(process_points, args)

    final_image = np.max(results, axis=0)
    final_image = enhance_image(final_image)

    origin_x = int((0 - p_min[0]) * x_ratio)
    origin_y = int((0 - p_min[1]) * y_ratio)
    print("Creating Image ...")
    for dx in range(-square_size // 2, square_size // 2 + 1):
        for dy in range(-square_size // 2, square_size // 2 + 1):
            nx, ny = origin_x + dx, origin_y + dy
            if 0 <= nx < img_width and 0 <= ny < img_height:
                final_image[ny, nx] = [1, 0, 0]

    final_image = np.fliplr(final_image)

    cv2.imwrite(save_path, cv2.cvtColor((final_image * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
    print(f"Image saved as {save_path}")

    save_data()  # Save the data after processing the point cloud

if __name__ == '__main__':
    load_data()  # Load data at startup
    load_processes()  # Load processes at startup
    start_roscore()
    rospy.init_node('odometry_listener', anonymous=True)
    rospy.Subscriber("/Odometry", Odometry, odometry_callback)
    start_cv_bridge_handler()
    app.run(host='0.0.0.0', debug=True, port=80)
S
