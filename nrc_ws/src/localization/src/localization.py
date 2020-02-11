#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from nrc_msgs.msg import LocalizationVector
import time
from math import cos, radians, sin

#Note: Localization puts localization into coordinates in meters

#topic /nrc/sensor_data

speed = 0
heading = 0
lastTime = 0
coordinates = [0,0]

def localization(sensor):
    global speed, heading, vectorPub, coordinates, lastTime

    speed = sensor.velocity
    heading = sensor.heading
    elapsedTime = time.time() - lastTime
    lastTime = time.time()

    localization_msg = LocalizationVector()

    localization_msg.x = speed * cos(heading) * elapsedTime
    localization_msg.y = speed * sin(heading) * elapsedTime

    coordinates[0] = coordinates[0] + localization_msg.x
    coordinates[1] = coordinates[1] + localization_msg.y

    localization_msg.x = coordinates[0]
    localization_msg.y = coordinates[1]

    vectorPub.publish(localization_msg)

    rospy.spin()

def mainLoop():
    rospy.init_node('nrc_localization_node')
    vectorPub = rospy.Publisher('/nrc/robot_state', LocalizationVector, queue_size=10)
    sensor = rospy.Subscriber("/nrc/sensor_data", DriveStatus, localization)
    lastTime = time.time()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        mainLoop()
    except rospy.ROSInterruptException:
        pass