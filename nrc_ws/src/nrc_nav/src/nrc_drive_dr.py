#! /usr/bin/env python

import json
import rospy
import serial

import time
import csv
from numpy import genfromtxt

from nrc_msgs.msg import DriveStatus, DriveCommand

# Global robot objects
robot_serial = None
status_pub = None


# def get_drivetrain_status(timer_event):
#     global robot_serial
#     global status_pub

#     # Read the status packet
#     status = robot_serial.read_until()

#     # Load status into JSON object
#     try:
#         status_json = json.loads(status)

#         # Package the status into a message
#         status_msg = DriveStatus()

#         status_msg.device_id = status_json['id']
#         status_msg.right_speed = status_json['right_vel']
#         status_msg.left_speed = status_json['left_vel']
#         status_msg.acceleration = status_json['acceleration']
#         status_msg.yaw = status_json['yaw']

#         # Publish the status
#         status_pub.publish(status_msg)

#     # Sometimes a ValueError occurs when you try to loads(). In this situation just try again next time
#     except ValueError:
#         pass



# def send_drivetrain_command(drive_cmd):
#     global robot_serial

#     # Form the command packet
#     command = {"heading": drive_cmd.heading, "speed": drive_cmd.speed}

#     # Convert the packet to a string
#     command_json = json.dumps(command)

#     # Write the command to serial
#     robot_serial.write(command_json)


def update_status(status_msg):
    # get local time at every tick to compare to the path-gen (in seconds)
    local_time = time.ctime(start_time)
    
    # csv is generated with path, based on time passed since start
    my_data = genfromtxt('my_file.csv', delimiter=',', skipheader=1, names="time,v,accel,hdg")
    # TODO check file to make sure column names are right
    # TODO figure out how to actually get the csv

    most_recent_time_index = round(truncate(local_time))
    most_recent_row = my_data[most_recent_time_index]

    recent_time = most_recent_row[0]
    vel = most_recent_row[1]
    accel = most_recent_row[2]
    hdg = most_recent_row[3]

    # use heading of most recent time in spreadsheet without interpolation
    new_heading = hdg
    # calculate desired velocity by taking most recent time's velocity and interpolating with accel and time
    new_velocity = vel + accel * (local_time - recent_time)

    command_msg = DriveCommand()
    command_msg.heading = new_heading
    command_msg.speed = new_velocity
    status_pub.publish(command_msg)


if __name__ == "__main__":
    # seconds passed since epoch
    start_time = time.time()
    # use this to get local_time at every tick

    # Initialize ROS node
    rospy.init_node("nrc_drive_dr")

    # Set up the serial communication to the robot
    robot_serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

    # Set up a publisher for publishing the status
    status_pub = rospy.Publisher("/nrc/cmd", DriveCommand, queue_size=1)

    # Set up a subscriber for getting commands
    command_sub = rospy.Subscriber("/nrc/cmd", DriveStatus, get_drivetrain_status, queue_size=1)

    # Set up a timer to read the sensor data at 200 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), update_status)

    # Pump callbacks
    rospy.spin()

    # Close the serial port when the program ends
    robot_serial.close()
