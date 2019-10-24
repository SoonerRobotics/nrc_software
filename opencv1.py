import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)

while True:
	_, frame = cap.read()

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#lower red mask
	lower_red = np.array([0,100,80])
	upper_red =np.array([20,255,255])
	mask0 = cv2.inRange(hsv,lower_red,upper_red)
	#upper red mask
	lower_red = np.array([160,100,80])
	upper_red = np.array([200,255,255])
	mask1 = cv2.inRange(hsv,lower_red,upper_red)

	mask = mask0 + mask1
	res = cv2.bitwise_and(frame,frame,mask=mask)

	cv2.imshow('frame', frame)
	cv2.imshow('MASKING', mask)
	cv2.imshow('ONLY_RED_COLOR_PASSED', res)
	key = cv2.waitKey(1)
	if key == 27:
		break
cv2.destroyAllWindows()


