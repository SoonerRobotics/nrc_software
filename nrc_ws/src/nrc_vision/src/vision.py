#!/usr/bin/env python

import cv2
import numpy as np
import visionSetup
import visionFunctions
import rospy
from nrc_msgs.msg import NodeVector

def talker():
    vectorPub = rospy.Publisher('/nrc/NodeVector', NodeVector, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #Robot prerun setup
    _, _, resHoop, resRamp, cntRamp = visionSetup.referenceImagesSetup()
    cap = cv2.VideoCapture(0)#enable video capture

    while not rospy.is_shutdown():
        _, frame = cap.read()
        frame = cv2.bilateralFilter(frame,9,75,75)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        maskStan, maskHoop, maskRamp = visionFunctions.getDefaultMasks(hsv)
        mask = maskStan + maskHoop + maskRamp
        res = cv2.bitwise_and(frame,frame,mask=mask)
        edges = cv2.Canny(mask, 75, 150)

        #Display an image.
        #THIS IS EXPENSIVE. FOR TESTING ONLY
        #cv2.imshow('edges', edges)
        #cv2.imshow('frame', frame)
        #cv2.imshow('MASKING', mask)
        #cv2.imshow('MASK APPLIED', res)

        #Contour Stuff goes here
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        rampDistance, rampAngle = visionFunctions.detectRamp(maskRamp, cntRamp)
        stanchions = visionFunctions.detectStan(edges)
        #hoopDistance, hoopAngle = visionFunctions.detectHoop(edges)
        
        #send found objects here
        nodeVector_msg = NodeVector()
        nodeVector_msg.rampDistance = rampDistance
        nodeVector_msg.rampAngle = rampAngle
        nodeVector_msg.stanchion1Distance = stanchions[0]["distance"]
        nodeVector_msg.stanchion1Angle = stanchions[0]["angle"]
        nodeVector_msg.stanchion2Distance = stanchions[1]["distance"]
        nodeVector_msg.stanchion2Angle = stanchions[1]["angle"]
        nodeVector_msg.stanchion3Distance = stanchions[2]["distance"]
        nodeVector_msg.stanchion3Angle = stanchions[2]["angle"]
        #nodeVector_msg.hoopDistance = hoopDistance
        #nodeVector_msg.hoopAngle = hoopAngle
        vectorPub.publish(nodeVector_msg)

        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
