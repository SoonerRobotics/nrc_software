import cv2
import numpy as np

def nothing(x):
	pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("Variables")

cv2.createTrackbar("L - H", "Variables", 0, 179, nothing)
cv2.createTrackbar("L - S", "Variables", 0, 255, nothing)
cv2.createTrackbar("L - V", "Variables", 0, 255, nothing)
cv2.createTrackbar("U - H", "Variables", 179, 179, nothing)
cv2.createTrackbar("U - S", "Variables", 255, 255, nothing)
cv2.createTrackbar("U - V", "Variables", 255, 255, nothing)


while True:
	_, frame = cap.read()

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	#mask
	l_h = cv2.getTrackbarPos("L - H", "Variables")
	l_s = cv2.getTrackbarPos("L - S", "Variables")
	l_v = cv2.getTrackbarPos("L - V", "Variables")
	u_h = cv2.getTrackbarPos("U - H", "Variables")
	u_s = cv2.getTrackbarPos("U - S", "Variables")
	u_v = cv2.getTrackbarPos("U - V", "Variables")

	lower = np.array([l_h,l_s,l_v])
	upper =np.array([u_h,u_s,u_v])
	mask = cv2.inRange(hsv,lower,upper)

	res = cv2.bitwise_and(frame,frame,mask=mask)

	cv2.imshow('frame', frame)
	cv2.imshow('MASKING', mask)
	cv2.imshow('FINAL RESULT', res)
	key = cv2.waitKey(1)
	if key == 27:
		break
cv2.destroyAllWindows()


