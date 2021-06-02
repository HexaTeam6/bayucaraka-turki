# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import numpy as np
import cv2
import os
#import dronekit
#from pymavlink import mavutil
import RPi.GPIO as GPIO

def positionServo (servo, angle):
    os.system("python angleServoCtrl.py " + str(servo) + " " + str(angle))
    print("[INFO] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

def nothing(x):
	pass

#create window
#cv2.namedWindow("image", cv2.WINDOW_NORMAL)
#cv2.createTrackbar('lowH','image',0,180,nothing)
#cv2.createTrackbar('lowS','image',125,255,nothing)
#cv2.createTrackbar('lowV','image',230,255,nothing)
#cv2.createTrackbar('highH','image',180,180,nothing)
#cv2.createTrackbar('highS','image',255,255,nothing)
#cv2.createTrackbar('highV','image',255,255,nothing)


# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("Inisiasi Kamera")
vs = PiVideoStream().start()
vs.camera.iso = 600
vs.camera.brightness = 56
time.sleep(15.0)

#print("Inisiasi Dronekit")
#vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True, baud=115200)

print("Inisiasi Servo")
positionServo(17,120)
positionServo(27,120)
#servoPIN = 17
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#GPIO.setup(servoPIN, GPIO.OUT)

#p = GPIO.PWM(servoPIN, 100) # GPIO 17 for PWM with 50Hz
#p.start(2.5) # Initialization

fps = FPS().start()
# loop over some frames...this time using the threaded stream

# lower mask (0-10)
lower_red = np.array([0,90,0])
upper_red = np.array([12,255,255])
#mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

# upper mask (170-180)
lower_red = np.array([160,90,0])
upper_red = np.array([180,255,255])
#mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

# join my masks
#mask = mask0+mask1

#servo counter
counter = 2
act = True

#while fps._numFrames < args["num_frames"]:
while(1):
	if cv2.waitKey(30) & 0xFF == 27:
		break
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=400)
	frame = cv2.flip(frame,-1)

	blurred = cv2.GaussianBlur(frame, (11,11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# get trackbar positions
	#ilowH = cv2.getTrackbarPos('lowH', 'image')
	#ihighH = cv2.getTrackbarPos('highH', 'image')
	#ilowS = cv2.getTrackbarPos('lowS', 'image')
	#ihighS = cv2.getTrackbarPos('highS', 'image')
	#ilowV = cv2.getTrackbarPos('lowV', 'image')
	#ihighV = cv2.getTrackbarPos('highV', 'image')

	#lower_hsv = np.array([ilowH, ilowS, ilowV])
	#higher_hsv = np.array([ihighH, ihighS, ihighV])

	mask0 = cv2.inRange(hsv, lower_red, upper_red)
	mask1 = cv2.inRange(hsv, lower_red, upper_red)

	# join my masks
	mask = mask0+mask1

	#mask = cv2.inRange(hsv, lower_hsv,higher_hsv)
	#mask =cv2.erode(mask,None, iterations=2)
	#mask = cv2.dilate(mask,None, iterations=2)
	mask2 = cv2.bitwise_and(frame,frame,mask=mask)
	
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x,y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		if M["m00"] != 0:
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		#area = cv2.contourArea(c)
		# count delta degree
		# if center[0] > 320:
		#     deg = (center[0] - 320) * horizontal_deg
		#     print("degree: " + str(deg))

		# area = 
		# if radius > 10:
		#print(area)
			if counter > 0 and act:
				act = False
				#do servo
				s = 0
				if(counter == 2):
					positionServo(17,30)
				else:
					positionServo(27,30)
				counter -= 1
				print(True)
			cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255),2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	else:
		act = True
		GPIO.cleanup()

	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		cv2.imshow("main2", mask2)
		#key = cv2.waitKey(1) & 0xFF

	# update the FPS counter
	fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
#positionServo(17,100)
#positionServo(27,100)
cv2.destroyAllWindows()
vs.stop()
#vehicle.close()
