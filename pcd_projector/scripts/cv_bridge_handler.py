#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
import os

bridge = CvBridge()
img_map = None
img_heatmap = None

def ros_image_callback_map(msg):
    global img_map
    try:
        img_map = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('/tmp/img_map.png', img_map)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error")

def ros_image_callback_heatmap(msg):
    global img_heatmap
    try:
        img_heatmap = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('/tmp/img_heatmap.png', img_heatmap)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error:")

if __name__ == '__main__':
    rospy.init_node('cv_bridge_handler', anonymous=True)
    rospy.Subscriber("/img_map", ROSImage, ros_image_callback_map)
    rospy.Subscriber("/img_heatmap", ROSImage, ros_image_callback_heatmap)
    rospy.spin()
