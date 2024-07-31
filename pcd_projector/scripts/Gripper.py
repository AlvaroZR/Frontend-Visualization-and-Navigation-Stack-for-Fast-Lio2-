#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool
import serial

class BoolToSerial:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('bool_to_serial', anonymous=True)
        
        # Get parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM1')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.topic_name = rospy.get_param('~topic_name', '/gripper')
        
        # Initialize serial connection
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        
        # Initialize state
        self.previous_value = None
        
        # Subscribe to the ROS topic
        rospy.Subscriber(self.topic_name, Bool, self.callback)
        
    def callback(self, msg):
        current_value = msg.data
        if current_value:
            self.ser.write('C')
        else:
            self.ser.write('O')
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bool_to_serial = BoolToSerial()
        bool_to_serial.run()
    except rospy.ROSInterruptException:
        pass
