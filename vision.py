import cv2
import numpy as np

HORIZONTAL_PIXELS = 640
FOV = 53#in degrees

cap = cv2.VideoCapture(0)#enable video capture

img2 = cv2.imread('redstar.jpg',-1)#this needs to be the target image shape we will use to compare to the image taken by the camera
	
hsvStar = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

#RED REQUIRES 2 MASKS FOR THE LOWER AND UPPER PART OF THE COLOR WHEEL
#lower red mask
lower_red = np.array([0,100,80])
upper_red = np.array([14,255,255])
maskStar0 = cv2.inRange(hsvStar,lower_red,upper_red)

#upper red mask
lower_red = np.array([164,100,80])
upper_red = np.array([179,255,255])
maskStar1 = cv2.inRange(hsvStar,lower_red,upper_red)

#apply the mask to the image
maskStar = maskStar0 + maskStar1
resStar = cv2.bitwise_and(img2,img2,mask=maskStar)

#find the contour of the target shape. For recognition we will match to this
_, contourStar,hierarchy = cv2.findContours(maskStar, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnt2 = contourStar[0]

while True:
	_, frame = cap.read()

	#blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

	#RED REQUIRES 2 MASKS FOR THE LOWER AND UPPER PART OF THE COLOR WHEEL	
	#lower red mask
	lower_red = np.array([0,100,80])
	upper_red =np.array([14,255,255])
	mask0 = cv2.inRange(hsv,lower_red,upper_red)

	#upper red mask
	lower_red = np.array([164,100,80])
	upper_red = np.array([179,255,255])
	mask1 = cv2.inRange(hsv,lower_red,upper_red)

	mask = mask0 + mask1
	res = cv2.bitwise_and(frame,frame,mask=mask)

	#Display an image.
	#THIS IS EXPENSIVE. FOR TESTING ONLY
	cv2.imshow('frame', frame)
	cv2.imshow('MASKING', mask)
	cv2.imshow('ONLY_RED_COLOR_PASSED', res)

	#Contour Stuff goes here
	_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	red = (0, 0, 255)
	#For every possible red shape in an image. Try to match it with redStar.jpg's red shape
	for contour in contours:
		area = cv2.contourArea(contour)
		#We do not want to create contours with random clutter. Use contours with a large number of pixels only
		if area > 600:#This value might be too big right now
			M = cv2.moments(contour)
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			#The center of a contour is (cX, cY)
			hull = cv2.convexHull(contour)
			cv2.drawContours(frame, hull, -1, (0,255,0), 3)
			cv2.drawContours(res, hull, -1, (0,255,0), 3)
			#Instead of drawing a literal line we will need to find the cX for the purpose of localization.
			comparisonA = cv2.matchShapes(contour,cnt2,1,0.0)		
			if comparisonA < .04:	#More testing may be needed for an ideal value here
				#print(comparisonA)#prints the comparison data for testing purposes
				#print(cX)#maybe send angle relative to robot instead?
				cv2.line(res, (cX,0), (cX,HORIZONTAL_PIXELS), red, thickness=3)
				angle = FOV - (cX/(HORIZONTAL_PIXELS/FOV))
				print(angle)
				#Need to implement send cX to localization

	#FOR TESTING ONLY
	cv2.imshow('frame', frame)
	cv2.imshow('ONLY_RED_COLOR_PASSED', res)
		
	#sleep(1)
	
	#loop exit condition. Wait for user to press esc key.
	key = cv2.waitKey(1)
	if key == 27:
		break
cv2.destroyAllWindows()
