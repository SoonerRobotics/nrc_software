#! /usr/bin/env python

import rospy
import rospkg

import time
import csv
from numpy import genfromtxt

from nrc_msgs.msg import DriveCommand

# Global robot object
command_pub = None
# global time
start_time = time.time()
# time index for trajectory gen instructions
instruction_index = 0
# instructions pulled in at the start of code running
instructions = None


def generate_drive_command(timer_event):
    # make sure we can access the global time and instruction_index
    global start_time
    global instruction_index
    global instructions
    global command_pub
    # get local time at every tick to compare to the path-gen (in seconds)
    local_time = time.time() - start_time - 15 #TEMP delay to account for time to start simulator

    if local_time <=0 or local_time > instructions[len(instructions) - 1][0] + 10:
        # don't start until the initial delay is passed (to wait for the sim to load)
        # and stop after the last instruction has been followed
        return

    # don't go past the end of the instructions
    # use local time to update most recent instruction
    while instruction_index < len(instructions)-1 and instructions[instruction_index + 1][0] < local_time:
        # if current time has passed the next instruction's time, switch "most recent" to it
        instruction_index += 1

    # pull variables we need from most recent instruction
    instruction_time = instructions[instruction_index][0]
    # can get x and y from positions 1 and 2 when needed
    vel = instructions[instruction_index][3]
    accel = instructions[instruction_index][4]
    hdg = instructions[instruction_index][5]

    # use heading of most recent instruction without interpolation
    new_heading = hdg
    # calculate desired velocity by taking most recent time's velocity and interpolating with accel and time
    new_velocity = vel + accel * (local_time - instruction_time)

    drive_cmd = DriveCommand()
    drive_cmd.heading = 360 - new_heading

    if new_heading > 355:
        # don't loop around to heading 0, but rather send 360
        drive_cmd.heading = new_heading

    drive_cmd.speed = new_velocity
    command_pub.publish(drive_cmd)


if __name__ == "__main__":
    # seconds passed since epoch (global time, used to find local time at every call)
    start_time = time.time()
    # initialize with first instruction
    instruction_index = 0

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('nrc_nav') + "/src/"

    # csv is generated with path, based on time passed since start
    # will need to make sure to copy file into this directory after creating it in trajectory_gen
    instructions = genfromtxt(filepath + 'output_traj.csv', delimiter=',', skip_header=1, names="time,x,y,velocity,accel,heading")

    # Initialize ROS node
    rospy.init_node("nrc_drive_dr")

    # Set up a publisher for publishing the drive command
    command_pub = rospy.Publisher("/nrc/path_cmd", DriveCommand, queue_size=1)

    # Set up a timer to read the sensor data at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), generate_drive_command)

    # Pump callbacks
    rospy.spin()
