#! /usr/bin/env python

## Simple publisher that publishes ROS Image data
## Publishes to the 'sniper_image' topic
## Westley Barragan and Jesse Wynn AUVSI '17

## TO DO:
## -maybe publish compressed image data instead??


import cv2
import numpy as np
import rospy
import roslib
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_capture():
    cap = cv2.VideoCapture(1)
    bridge = CvBridge()
    #publish to topic 'sniper_image'
    pub = rospy.Publisher('sniper_image', Image, queue_size=1)
    rospy.init_node('sniper_image_publisher', anonymous=True)
    rate=rospy.Rate(2)
    while not rospy.is_shutdown():
        grabbed, frame = cap.read()
        try:
            pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)
        rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        image_capture()
    except rospy.ROSInterruptException:
        pass
