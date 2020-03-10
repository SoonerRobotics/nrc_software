#! /usr/bin/env python

import json
import rospy
import serial

from nrc_msgs.msg import DriveStatus, motors
from sensor_msgs.msg import Joy

# Global robot objects
drive_cmd = None
status_pub = None

def on_joy(joy_cmd):
    msg = motors()
    left = joy_cmd.axes[1]
    right = joy_cmd.axes[4]

    msg.left = right * 2
    msg.right = left * 2

    # Convert the packet to a string
    drive_cmd.publish(msg)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("nrc_manual_node")

    # Set up the serial communication to the robot
    drive_cmd = rospy.Publisher("/nrc/motors", motors, queue_size=1)

    # Set up a subscriber for getting commands
    command_sub = rospy.Subscriber("/joy", Joy, on_joy, queue_size=1)

    # Pump callbacks
    rospy.spin()
