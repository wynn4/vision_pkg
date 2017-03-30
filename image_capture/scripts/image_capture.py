#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import roslib
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




 		self.bridge = CvBridge()


 	def image_caputure_publish(self):


 		grabbed, frame = self.c.read()












def image_capture():
	c = cv2.VideoCapture(0)
	bridge = CvBridge()
	pub = rospy.Publisher('image_capture', Image, queue_size=10)
	rospy.init_node('image_capture', anonymous=True)
	rate=rospy.Rate(15)
	while not rospy.is_shutdown():
		grabbed, frame = c.read()
		try:
			pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
		except CvBridgeError as e:
			print(e)
		rate.sleep()

if __name__ == '__main__':
	try:
		image_capture()
	except rospy.ROSInterruptException:
		pass





