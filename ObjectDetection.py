import numpy as np
import cv2
import math
import imutils
from imutils.video import FPS

def nothing(x):
    pass

horizontal_deg = 0.08359375

#Create Trackbar
cv2.namedWindow('image',cv2.WINDOW_NORMAL)
cv2.createTrackbar('lowH','image',0,180,nothing)
cv2.createTrackbar('lowS','image',125,255,nothing)
cv2.createTrackbar('lowV','image',230,255,nothing)
cv2.createTrackbar('highH','image',180,180,nothing)
cv2.createTrackbar('highS','image',255,255,nothing)
cv2.createTrackbar('highV','image',255,255,nothing)
cv2.createTrackbar('index','image',5,10,nothing)
cv2.createTrackbar('height','image',1,100,nothing)
red_lower = (0,70,50)
red_upper = (20,255,255)

#Initialize camera
#cv2.VideoCapture()
cam = cv2.VideoCapture(0)
fps = FPS().start()

while(cam.isOpened()):
	if cv2.waitKey(30) & 0xFF == 27:
		break

	ret, img = cam.read()
	width  = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
	img = cv2.flip(img,1)

	blurred = cv2.GaussianBlur(img, (11,11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# get trackbar positions
	ilowH = cv2.getTrackbarPos('lowH', 'image')
	ihighH = cv2.getTrackbarPos('highH', 'image')
	ilowS = cv2.getTrackbarPos('lowS', 'image')
	ihighS = cv2.getTrackbarPos('highS', 'image')
	ilowV = cv2.getTrackbarPos('lowV', 'image')
	ihighV = cv2.getTrackbarPos('highV', 'image')

	lower_hsv = np.array([ilowH, ilowS, ilowV])
	higher_hsv = np.array([ihighH, ihighS, ihighV])

	mask = cv2.inRange(hsv, lower_hsv,higher_hsv)
	mask =cv2.erode(mask,None, iterations=2)
	mask = cv2.dilate(mask,None, iterations=2)
	mask2 = cv2.bitwise_and(img,img,mask=mask)

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x,y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		area = cv2.contourArea(c)
		# count delta degree
		if center[0] > 320:
			deg = (center[0] - 320) * horizontal_deg
			print("degree: " + str(deg))

		# area = 
		if radius > 10:
			print(area)
		cv2.circle(img, (int(x), int(y)), int(radius), (0,255,255),2)
		cv2.circle(img, center, 5, (0, 0, 255), -1)

	cv2.imshow("Main",img)
	cv2.imshow("Main2",mask2)
	fps.update()

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

cam.release()
cv2.destroyAllWindows()
