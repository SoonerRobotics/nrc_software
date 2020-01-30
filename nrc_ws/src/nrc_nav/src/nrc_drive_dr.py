#! /usr/bin/env python

#import json
import rospy
#import serial

import time
import csv
from numpy import genfromtxt

from nrc_msgs.msg import DriveCommand

# Global robot objects
robot_serial = None
command_pub = None

# global time
start_time = time.time()
# time index for trajectory gen instructions
instruction_index = 0
# instructions pulled in at the start of code running
instructions = None



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


def generate_drive_command(timer_event):
    # get local time at every tick to compare to the path-gen (in seconds)
    local_time = time.ctime(start_time)

    # use local time to update most recent instruction
    while instructions[instruction_index + 1][0] < local_time:
        # if current time has passed the next instruction's time, switch "most recent" to it
        instruction_index += 1

    # pull variables we need from most recent instruction
    instruction_time = my_data[instruction_index][0]
    # can get x and y from positions 1 and 2 when needed
    vel = my_data[instruction_index][3]
    accel = my_data[instruction_index][4]
    hdg = my_data[instruction_index][5]

    # use heading of most recent instruction without interpolation
    new_heading = hdg
    # calculate desired velocity by taking most recent time's velocity and interpolating with accel and time
    new_velocity = vel + accel * (local_time - instruction_time)

    drive_cmd = DriveCommand()
    drive_cmd.heading = new_heading
    drive_cmd.speed = new_velocity
    command_pub.publish(drive_cmd)


if __name__ == "__main__":
    # seconds passed since epoch (global time, used to find local time at every call)
    start_time = time.time()

    # csv is generated with path, based on time passed since start
    # TODO find exact filepath or make sure traj gen outputs the file to this directory. 
    #       for now, I copied the file into this directory manually.
    instructions = genfromtxt('output_traj.csv', delimiter=',', skipheader=1, names="time,x,y,velocity,accel,heading")

    # Initialize ROS node
    rospy.init_node("nrc_drive_dr")

    # TODO does this code need robot_serial or command_sub?
    # Set up the serial communication to the robot
    #robot_serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

    # Set up a subscriber for getting commands
    #command_sub = rospy.Subscriber("/nrc/cmd", DriveStatus, get_drivetrain_status, queue_size=1)

    # Set up a publisher for publishing the drive command
    command_pub = rospy.Publisher("/nrc/cmd", DriveCommand, generate_drive_command, queue_size=1)

    # Set up a timer to read the sensor data at 200 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), update_status) #TODO change update_status to generate_drive_command?

    # Pump callbacks
    rospy.spin()

    # Close the serial port when the program ends
    robot_serial.close()
