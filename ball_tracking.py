#!/usr/bin/env python3
from __future__ import print_function
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from collections import deque
import imutils
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def talker(image):
	#Setup ROS publish topic
	pub=rospy.Publisher('coordinates',Float32,queue_size=1)
	pub2=rospy.Publisher('rotation_angle',Float32,queue_size=1)
	rate = rospy.Rate(20) # 10hz

	#Setup color filter
	greenLower = (35, 100, 100)
	greenUpper = (64, 255, 255)
	pts = deque(maxlen=64)

	#Start processing images
	np_arr = np.fromstring(image.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	frame = np.array(image_np, dtype=np.uint8)
	frame = imutils.resize(frame, width=600)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print(center)
		#compute distance
		distance_from_center=int(M["m01"] / M["m00"])
		pub.publish(distance_from_center)
		#compute angle
		distance_per_degree=4
		rotation_angle=x/distance_per_degree-80
		pub2.publish(rotation_angle)
		#log distance & angle
		rospy.loginfo("Distance from center: "+str(distance_from_center))
		rospy.loginfo("Angle from center:" + str(rotation_angle))
		rate.sleep()
		#draw the ball on the window
		if radius > 10:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	pts.appendleft(center)
	for i in range(1, len(pts)):
		if pts[i - 1] is None or pts[i] is None:
			continue
		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
	image_pub.publish(img_msg)
	#cv2.imshow("Frame", frame)


try:
	print('Start Green Ball Tracking...')
	rospy.init_node('ball_tracking', anonymous=True)
	image_pub = rospy.Publisher("/output",Image)
	bridge = CvBridge()
	image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,talker,queue_size = 1, buff_size=2**24)
	rospy.spin()
except KeyboardInterrupt:
	print("Shutting down")
	cv2.destroyAllWindows()
