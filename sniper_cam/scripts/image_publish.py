#! /usr/bin/env python			

##Simple publisher that publishes ROS Image data
## Publishes to the 'interop_images' topic
## Gmoney AUVSI '17
	
## TO DO:
## Publish final images as a ROS topic that interop can use
	
	
import cv2
import numpy as np					
import rospy
import roslib
import sys
import numpy as np					
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sniper_cam.msg import interopImages



from cv_bridge import CvBridge, CvBridgeError		

def image_transfer():

	#Publish to 'interop_images'
	pub = rospy.Publisher('plans', interopImages, queue_size =  10) 	
	bridge = CvBridge()
	msg = interopImages()

	rospy.init_node('death_star', anonymous=True)
	rate = rospy.Rate(1)					#One image/sec


#	f1 = open("Read In/image.jpg")				
	f2 = open("Read In/lat.txt")				
	f3 = open("Read In/longi.txt")				
	f4 = open("Read In/target_color.txt")			
	f5 = open("Read In/target_shape.txt")			
	f6 = open("Read In/symbol.txt")				#Shape File
	f7 = open("Read In/symbol_color.txt")			#Shape Color File
	f8 = open("Read In/orientation.txt")			

#	image = f1.read			#Image file
	lat = f2.readline()				#Lat file
	longi = f3.readline()				#Longi File
	target_color = f4.readline()			#Color File
	target_shape = f5.readline()			#Shape File
	symbol = f6.readline()
	symbol_color = f7.readline()
	orientation = f8.readline()			#Orientation file

	image = cv2.imread("Read In/image.jpg")
	try:
	    image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
	except CvBridgeError as e:
	    print(e)

#	msg.image = image_msg
	msg.gps_lati = float(lat)
	msg.gps_longit = float(longi)
	msg.target_color = target_color
	msg.target_shape = target_shape
	msg.symbol = symbol
	msg.symbol_color = symbol_color
	msg.orientation = orientation
	
	#print(type(float(lat)))
	#print(msg)
	

	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
		try:
			image_transfer()
		except rospy.ROSInterruptException: pass
	
	
