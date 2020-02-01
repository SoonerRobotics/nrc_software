#! /usr/bin/env python

import rospy
import rospkg

import time
import csv
from numpy import genfromtxt

from nrc_msgs.msg import DriveCommand
from igvc_msgs.msg import velocity, imuodom, motors

# Global variables
# keep most recent commanded heading and speed
cmd_heading = 0
cmd_speed = 0
# keep current robot position/info
cur_leftVel = None
cur_rightVel = None
cur_accel = None
cur_heading = None
# define motor publisher
motor_pub = None

def receive_drive_command(drive_cmd):
    global cmd_heading
    global cmd_speed
    # keep the most recent commanded heading and speed
    cmd_heading = drive_cmd.heading
    cmd_speed = drive_cmd.speed

def receive_velocity(cur_vel):
    global cur_leftVel
    global cur_rightVel
    # keep track of current velocity
    cur_left_vel = cur_vel.leftVel
    cur_right_vel = cur_vel.rightVel

def receive_imu_data(imu_data):
    global cur_accel
    global cur_heading
    # keep track of accel and heading
    cur_accel = imu_data.acceleration
    cur_heading = imu_data.heading

def send_motor_cmd(timer_event):
    # bring in all the globals
    global cmd_heading, cmd_speed, cur_leftVel, cur_rightVel
    global cur_accel, cur_heading, motor_pub
    # create the motor command
    motor_cmd = motors()

    turn_power = (cmd_heading - cur_heading) * 0.2
    drive_power = (cmd_speed - (cur_leftVel + cur_rightVel)/2) * 0.3

    # set the left and right motor velocities
    motor_cmd.left = drive_power - turn_power
    motor_cmd.right = drive_power + turn_power

    # publish the motor command
    motor_pub.publish(motor_cmd)


if __name__ == "__main__":
    # seconds passed since epoch (global time, used to find local time at every call)
    start_time = time.time()
    
    # Initialize ROS node
    rospy.init_node("sim_output")

    # Set up subscribers for getting commands and info
    command_sub = rospy.Subscriber("/nrc/cmd", DriveCommand, receive_drive_command, queue_size=1)
    vel_sub = rospy.Subscriber("/nrc/velocity", velocity, receive_velocity, queue_size=1)
    imu_sub = rospy.Subscriber("/nrc/imu", imuodom, receive_imu_data, queue_size=1)

    # Set up a publisher for publishing to the motors 
    motor_pub = rospy.Publisher("/nrc/motors", motors, queue_size=1)

    # Set up a timer to read the sensor data at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_motor_cmd)

    # Pump callbacks
    rospy.spin()
