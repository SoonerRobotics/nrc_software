#! /usr/bin/env python

import rospy
import rospkg

import time
import csv
from numpy import genfromtxt

from nrc_msgs.msg import DriveCommand, DriveStatus, Motors

# Global variables
# keep most recent commanded heading and speed
cmd_heading = 0
cmd_speed = 0
# define command publisher
command_pub = None

def receive_drive_command(drive_cmd):
    global cmd_heading
    global cmd_speed
    # keep the most recent commanded heading and speed
    cmd_heading = drive_cmd.heading
    cmd_speed = drive_cmd.speed

def send_command(status):
    # bring in all the globals
    global cmd_heading, cmd_speed

    # Determine desired left and right motor speed
    turn_power = (cmd_heading - status.yaw) * 0.2
    drive_power = (cmd_speed - (status.left_speed + status.right_speed)/2) * 0.3

    # Create Motors command to set left and right motor speed
    motors = Motors()
    motors.left = drive_power - turn_power
    motors.right = drive_power + turn_power

    # publish the motor command
    command_pub.publish(motors)

if __name__ == "__main__":
    # seconds passed since epoch (global time, used to find local time at every call)
    start_time = time.time()
    
    # Initialize ROS node
    rospy.init_node("sim_output")

    # Set up subscribers for getting commands and info
    command_sub = rospy.Subscriber("/nrc/path_cmd", DriveCommand, receive_drive_command, queue_size=1)
    status_sub = rospy.Subscriber("/nrc/sensor_data", DriveStatus, send_command, queue_size=1)

    # Set up a publisher for publishing to the motors
    command_pub = rospy.Publisher("/nrc/motors", Motors, queue_size=1)

    # Pump callbacks
    rospy.spin()
