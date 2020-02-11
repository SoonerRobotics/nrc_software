#Use for setting up default image references
import cv2
import numpy as np

#Default reference images are loaded here
def referenceImagesSetup():
    resStan, cntStan = refStanchion()
    resHoop = refHoop()
    resRamp, cntRamp = refRamp()
    return resStan, cntStan, resHoop, resRamp, cntRamp

#Default red stanchion
def refStanchion():
    #REFERENCE IMAGES WITH THIS IMPLEMENTATION REQUIRE A DIRECT PATH
    img0 = cv2.imread('/home/david/nrc_software/nrc_ws/src/nrc_vision/src/redstanchion.JPG',-1)#this needs to be the target image shape we will use to compare to the image taken by the camera
    hsvStan = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)

    #RED REQUIRES 2 MASKS FOR THE LOWER AND UPPER PART OF THE COLOR WHEEL
    #lower red mask
    lower_red = np.array([0,100,80])
    upper_red = np.array([19,255,255])
    maskStan0 = cv2.inRange(hsvStan,lower_red,upper_red)

    #upper red mask
    lower_red = np.array([159,100,80])
    upper_red = np.array([179,255,255])
    maskStan1 = cv2.inRange(hsvStan,lower_red,upper_red)

    #apply the mask to the image
    maskStan = maskStan0 + maskStan1
    resStan = cv2.bitwise_and(img0,img0,mask=maskStan)

    #find the contour of the target shape. For recognition we will match to this
    _, contourStan,hierarchy = cv2.findContours(maskStan, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntStan = contourStan[0]

    #For testing the reference image
    #cv2.imshow("reference", resStan)

    return resStan, cntStan

#default green hoop
#This can be edited later since line detection will work better for the hoop
def refHoop():
    #REFERENCE IMAGES WITH THIS IMPLEMENTATION REQUIRE A DIRECT PATH
    imgHoop = cv2.imread('/home/david/nrc_software/nrc_ws/src/nrc_vision/src/greenhoop.JPG', -1)
    hsvHoop = cv2.cvtColor(imgHoop, cv2.COLOR_BGR2HSV)

    lower_green = np.array([50,100,80])
    upper_green = np.array([70,255,255])
    maskHoop = cv2.inRange(hsvHoop, lower_green, upper_green)
    resHoop = cv2.bitwise_and(imgHoop,imgHoop,mask=maskHoop)

    #For testing the reference image
    #cv2.imshow("reference", resHoop)

    return resHoop

#default blue ramp
def refRamp():
    #REFERENCE IMAGES WITH THIS IMPLEMENTATION REQUIRE A DIRECT PATH
    imgRamp = cv2.imread('/home/david/nrc_software/nrc_ws/src/nrc_vision/src/blueramp.JPG', -1)
    hsvRamp = cv2.cvtColor(imgRamp, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([105,100,80])
    upper_blue = np.array([125,255,255])
    maskRamp = cv2.inRange(hsvRamp, lower_blue, upper_blue)
    resRamp = cv2.bitwise_and(imgRamp,imgRamp,mask=maskRamp)

    _, contourRamp,hierarchy = cv2.findContours(maskRamp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntRamp = contourRamp[0]

    #For testing the reference image
    #cv2.imshow("reference", resRamp)

    return resRamp, cntRamp