#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from nrc_msgs.msg import LocalizationVector, DriveStatus
import time
from math import cos, radians, sin

#Note: Localization puts localization into coordinates in meters

#topic /nrc/sensor_data

speed = 0
heading = 0
last_time = 0
coordinates = [0,-1]
vector_pub = None

def localization_cb(sensor):
    global speed, heading, vector_pub, coordinates, last_time

    linear_speed = (sensor.right_speed + sensor.left_speed) / 2
    # convert the angular speed to linear
    wheel_radius = 0.0635 # in meters, just a guess of 2.5 inches
    # width of axle is 0.28 meters (doesn't matter for now but who knows)


    heading = sensor.yaw
    elapsed_time = time.time() - last_time
    last_time = time.time()

    localization_msg = LocalizationVector()

    #calculated the delta for distance since last cycle
    localization_msg.x = linear_speed * cos(radians(heading)) * elapsed_time
    localization_msg.y = linear_speed * sin(radians(heading)) * elapsed_time * -1

    coordinates[0] = coordinates[0] + localization_msg.x
    coordinates[1] = coordinates[1] + localization_msg.y

    localization_msg.x = coordinates[0]
    localization_msg.y = coordinates[1]

    vector_pub.publish(localization_msg)

def main_loop():
    global vector_pub, last_time

    rospy.init_node('nrc_localization_node')
    vector_pub = rospy.Publisher('/nrc/robot_state', LocalizationVector, queue_size=10)
    sensor_sub = rospy.Subscriber("/nrc/sensor_data", DriveStatus, localization_cb, queue_size=1)
    last_time = time.time()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
