#This will be used by the vision module after setup and midrun
import cv2
import numpy as np

HORIZONTAL_PIXELS = 640
FOV = 53#in degrees
VERTICAL_PIXELS = 480

def smallAngle(e):
    return e["angle"]

def getDefaultMasks(hsvLive):
    #RED STANCHION
    #lower red mask
    lower_red = np.array([0,100,80])
    upper_red = np.array([19,255,255])
    maskStan0 = cv2.inRange(hsvLive,lower_red,upper_red)

    #upper red mask
    lower_red = np.array([159,100,80])
    upper_red = np.array([179,255,255])
    maskStan1 = cv2.inRange(hsvLive,lower_red,upper_red)

    #apply the mask to the image
    maskStan = maskStan0 + maskStan1

    #GREEN HOOP
    lower_green = np.array([50,100,80])
    upper_green = np.array([70,255,255])
    maskHoop = cv2.inRange(hsvLive, lower_green, upper_green)

    #BLUE RAMP
    lower_blue = np.array([105,100,80])
    upper_blue = np.array([125,255,255])
    maskRamp = cv2.inRange(hsvLive, lower_blue, upper_blue)

    return maskStan, maskHoop, maskRamp

def detectStan(edges):
    detectedStanchions = []
    lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength=200,maxLineGap=10)
    noOfLines = 0
    try:
        for line in lines or ():
            x1,y1,x2,y2 = line[0]
            if abs(x1-x2) > 15:
                continue
            else: #valid stanchion detected
                if x2-x1 == 0:
                    length = y1-y2
                    mid = x1;
                else:
                    length = (y1-y2)/(x1-x2)
                    mid = (x1+x2)/2
                distance = (VERTICAL_PIXELS / length) * 53
                angle = FOV - (mid/(HORIZONTAL_PIXELS/FOV))
                detectedStanchion = {
                    "distance": distance,
                    "angle": angle
                }
                detectedStanchions.append(detectedStanchion)
                noOfLines = noOfLines + 1
    except:
        print("no lines")
    detectedStanchions.sort(key=smallAngle)
    if noOfLines < 3:
        while noOfLines < 3:
            detectedStanchion = {
                "distance": -1,
                "angle": -1
            }
            detectedStanchions.append(detectedStanchion)
            noOfLines = noOfLines + 1
    return detectedStanchions
            
    
#This will be used to detect an approaching ramp

def detectRamp(mask, cntRamp):
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours or ():
        area = cv2.contourArea(contour)
        #We do not want to create contours with random clutter. Use contours with a large number of pixels only
        if area > 500:#This value might be too big right now
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #The center of a contour is (cX, cY)

            comparisonA = cv2.matchShapes(contour,cntRamp,1,0.0)

            if comparisonA < .08:	#More testing may be needed for an ideal value here
                angle = FOV - (cX/(HORIZONTAL_PIXELS/FOV))
                rightmost = tuple(contour[contour[:,:,0].argmin()][0])
                leftmost = tuple(contour[contour[:,:,0].argmin()][0])
                if rightmost < HORIZONTAL_PIXELS and leftmost > 0:
                    distance = (HORIZONTAL_PIXELS / (rightmost - leftmost)) * 45
                elif leftmost == 0:
                    angle = FOV
                    distance = -1
                else: #Using the vertical pixels of the ramp will be more inaccurate
                    angle = 0
                    distance = -1 
                return distance, angle
    return -1, -1

#This needs to be fixed still
def detectHoop():
    return -1