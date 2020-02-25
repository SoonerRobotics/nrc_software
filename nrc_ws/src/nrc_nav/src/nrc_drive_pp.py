#! /usr/bin/env python

import rospy, rospkg

import csv
from math import atan2, pi
from numpy import genfromtxt

from pure_pursuit import PurePursuit
from nrc_msgs.msg import DriveCommand, LocalizationVector, DriveStatus, Motors

command_pub = None
# generated trajectory, pulled in once at start
instructions = None
# pure pursuit path
pp = PurePursuit()
# current position and heading
pos = None
heading = None

def generate_pure_pursuit_path():
    global pp
    pp = PurePursuit()
    for i in range(len(instructions)):
        # add x,y coords from each point in the generated trajectory as waypoints.
        # this is better than just adding the 5 nodes as waypoints.
        pp.add_point(instructions[i][1], instructions[i][2])

def receive_position(local_pos):
    # triggers when receiving position from David's localization code
    global pos
    pos = (local_pos.x, local_pos.y)

def receive_heading(status):
    # triggers when sensors (IMU) publish sensor data, including yaw
    global heading
    heading = status.yaw


def generate_motor_command(timer_event): 
    # this function template was taken from igvc_nav_node.py
    if pos is None or heading is None:
        # wait until sensors/localization bring in data to do anything
        return

    # declare the look-ahead point
    lookahead = None
    # start with a search radius of 0.4 meters
    radius = 0.4 

    # look until finding the path at the increasing radius or hitting 2 meters
    while lookahead is None and radius <= 2: 
        lookahead = pp.get_lookahead_point(pos[0], pos[1], radius)
        radius *= 1.25
    
    # make sure we actually found the path
    if lookahead is not None:
        heading_to_la = 90 - atan2(lookahead[1] - pos[1], lookahead[0] - pos[0]) * 180 / (pi)
        if heading_to_la < 0:
            heading_to_la += 360

        delta = heading_to_la - heading
        delta = (delta + 180) % 360 - 180

        # make the motors command
        motor_msg = Motors()
        motor_msg.left = 2 + 1 * (delta / 180)
        motor_msg.right = 2 - 1 * (delta / 180)
        
        command_pub.publish(motor_msg)

if __name__ == "__main__":
    # initialize with first instruction
    instruction_index = 0

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('nrc_nav') + "/src/"

    # csv is generated with path, based on time passed since start
    # will need to make sure to copy file into this directory after creating it in trajectory_gen
    instructions = genfromtxt(filepath + 'output_traj.csv', delimiter=',', skip_header=1, names="time,x,y,velocity,accel,heading")

    # create the pure pursuit path using the generated trajectory
    generate_pure_pursuit_path()

    # get localization info from David's code
    local_sub = rospy.Subscriber("/nrc/robot_state", LocalizationVector, receive_position, queue_size=1)
    # get heading from a DriveStatus
    status_sub = rospy.Subscriber("/nrc/sensor_data", DriveStatus, receive_heading, queue_size=1)

    # Initialize ROS node
    rospy.init_node("nrc_drive_dr")

    # Set up a publisher for publishing the drive command
    #command_pub = rospy.Publisher("/nrc/path_cmd", DriveCommand, queue_size=1)
    command_pub = rospy.Publisher("/nrc/motors", Motors, queue_size=1)

    # Set up a timer to generate new commands at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), generate_motor_command)

    # Pump callbacks
    rospy.spin()
