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
lastTime = 0
coordinates = [0,-1]
vectorPub = None

def localization_cb(sensor):
    global speed, heading, vectorPub, coordinates, lastTime
    print("yeet")

    speed = (sensor.right_speed + sensor.left_speed) / 2
    heading = sensor.yaw
    elapsedTime = time.time() - lastTime
    lastTime = time.time()

    localization_msg = LocalizationVector()

    localization_msg.x = speed * cos(radians(heading)) * elapsedTime
    localization_msg.y = speed * sin(radians(heading)) * elapsedTime * -1

    coordinates[0] = coordinates[0] + localization_msg.x
    coordinates[1] = coordinates[1] + localization_msg.y

    localization_msg.x = coordinates[0]
    localization_msg.y = coordinates[1]

    vectorPub.publish(localization_msg)

def mainLoop():
    global vectorPub, lastTime

    rospy.init_node('nrc_localization_node')
    vectorPub = rospy.Publisher('/nrc/robot_state', LocalizationVector, queue_size=10)
    sensor_sub = rospy.Subscriber("/nrc/sensor_data", DriveStatus, localization_cb, queue_size=1)
    lastTime = time.time()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        mainLoop()
    except rospy.ROSInterruptException:
        pass