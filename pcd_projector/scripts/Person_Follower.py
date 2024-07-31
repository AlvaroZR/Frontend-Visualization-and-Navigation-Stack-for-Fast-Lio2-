#!/usr/bin/env python3.11

from pathlib import Path
import cv2
import depthai as dai
import serial
import time
import argparse
import blobconverter

# Constants
SHOW_PREVIEW = False  # Set to False to disable preview on the computer
SAVE_INTERVAL = 0.2  # Save image at 5 fps
IMG_MAP_PATH = '/tmp/img_camera.png'

MAX_SPEED = 16
MIN_SPEED = -16
BASE_TURN_SPEED = 10
BACKWARD_SPEED = -10
FORWARD_SPEED = 14
KP = 25  # Ganancia de control de movimiento lineal
KT = 10  # Ganancia de control de giro


labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]


nnPathDefault = blobconverter.from_zoo(
        name="mobilenet-ssd",
        shaves=5,
        zoo_type="intel"
    )
parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-ff', '--full_frame', action="store_true", help="Perform tracking on full RGB frame", default=False)

args = parser.parse_args()

fullFrameTracking = args.full_frame

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
objectTracker = pipeline.create(dai.node.ObjectTracker)

xoutRgb = pipeline.create(dai.node.XLinkOut)
trackerOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("preview")
trackerOut.setStreamName("tracklets")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

# setting node configs
stereo.initialConfig.setConfidenceThreshold(255)

spatialDetectionNetwork.setBlobPath(args.nnPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(8000)

objectTracker.setDetectionLabelsToTrack([15])  # track only person
# possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
# take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

# dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
objectTracker.out.link(trackerOut.input)

if fullFrameTracking:
    camRgb.setPreviewKeepAspectRatio(False)
    camRgb.video.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    # do not block the pipeline if it's too slow on full frame
    objectTracker.inputTrackerFrame.setQueueSize(2)
else:
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)
stereo.depth.link(spatialDetectionNetwork.inputDepth)


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
        # print(f"Sent: {message}")
    else:
        print("Serial port not open or not initialized.")

def format_output(left_motor, right_motor):
    left_direction = 1 if left_motor >= 0 else 0
    right_direction = 1 if right_motor >= 0 else 0

    left_speed = abs(left_motor)
    right_speed = abs(right_motor)

    left_brake = 1 if left_speed > 0 else 0
    right_brake = 1 if right_speed > 0 else 0

    output = "{:01d}{:03d}{:01d}{:01d}{:03d}{:01d}".format(left_direction, left_speed, left_brake, right_direction, right_speed, right_brake)
    return output

port = '/dev/ttyACM0'
baudrate = 115200
ser = initialize_serial(port, baudrate)
time.sleep(2)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", 4, False)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    last_save_time = time.monotonic()


    while(True):
        imgFrame = preview.get()
        track = tracklets.get()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        for t in trackletsData:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)

            try:
                label = labelMap[t.label]
            except:
                label = t.label

            left_motor = 0
            right_motor = 0

            if t.status.name == "TRACKED":
                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                cv2.putText(frame, f"X: {int(t.spatialCoordinates.x) / 1000} m", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y) / 1000} m", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z) / 1000} m", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))

                x_coord = int(t.spatialCoordinates.x) / 1000
                z_coord = int(t.spatialCoordinates.z) / 1000

                if z_coord < 2 and x_coord > 0.200:  # Operator is to the left of the robot
                    turn_correction = KT * x_coord
                    left_motor = -BASE_TURN_SPEED - turn_correction
                    right_motor = BASE_TURN_SPEED + turn_correction

                    # Ensure the motor speeds are within allowable range
                    left_motor = max(min(left_motor, MAX_SPEED), MIN_SPEED)
                    right_motor = max(min(right_motor, MAX_SPEED), MIN_SPEED)

                elif z_coord < 2 and x_coord < -0.200:  # Operator is to the right of the robot
                    turn_correction = KT * x_coord
                    left_motor = BASE_TURN_SPEED - turn_correction
                    right_motor = -BASE_TURN_SPEED + turn_correction

                    # Ensure the motor speeds are within allowable range
                    left_motor = max(min(left_motor, MAX_SPEED), MIN_SPEED)
                    right_motor = max(min(right_motor, MAX_SPEED), MIN_SPEED)

                elif z_coord >= 2:  # Operator is more than 2 meters away
                    # Move forward with proportional steering adjustment
                    forward_correction = KP * x_coord
                    left_motor = FORWARD_SPEED - forward_correction
                    right_motor = FORWARD_SPEED + forward_correction

                    # Ensure the motor speeds are within allowable range
                    left_motor = max(min(left_motor, MAX_SPEED), MIN_SPEED)
                    right_motor = max(min(right_motor, MAX_SPEED), MIN_SPEED)

                elif 0.2 <= z_coord <= 1.5:  # Operator is less than 1 meter away
                    backward_correction = KP * x_coord
                    left_motor = BACKWARD_SPEED - backward_correction
                    right_motor = BACKWARD_SPEED + backward_correction

                    # Ensure the motor speeds are within allowable range
                    left_motor = max(min(left_motor, MAX_SPEED), MIN_SPEED)
                    right_motor = max(min(right_motor, MAX_SPEED), MIN_SPEED)
                else:  # Operator is within 1 to 2 meters and centered
                    left_motor = 0
                    right_motor = 0

            else:
                left_motor = 0
                right_motor = 0

            output = format_output(int(left_motor), int(right_motor))
            send_message(ser, output)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

        # Save the frame to a file at 5 fps
        if current_time - last_save_time >= SAVE_INTERVAL:
            cv2.imwrite(IMG_MAP_PATH, frame)
            last_save_time = current_time

        if SHOW_PREVIEW:
            cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == 27:
            break
