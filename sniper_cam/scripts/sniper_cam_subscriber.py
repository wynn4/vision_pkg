#! /usr/bin/python

## Simple subscriber that subscribes to ROS Image data
## Subscribes to the 'sniper_image' topic
## Westley Barragan and Jesse Wynn AUVSI '17

## TO DO:
## -figure out how we want to save the images
## -maybe subscribe to compressed image data??

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

VERBOSE = False
# create CvBridge object
bridge = CvBridge()
global_var={'image_n':0}
globals().update(global_var)

def image_callback(msg):

    if VERBOSE:
        print 'recieved image of type: "%s"' % msg.format

    # direct conversion to CV2

    np_arr = np.fromstring(msg.data, np.uint8)
    img_np = cv2.imdecode(np_arr, 1)

    cv2.imshow('recieved image', img_np)
    cv2.imwrite('images/image' + str(image_n)+ '.jpg', img_np)
    global image_n
    image_n +=1
    cv2.waitKey(1)


def image_subscriber():
    rospy.init_node('sniper_image_subscriber')
    # Subscribe to 'sniper_image' topic and define its callback
    rospy.Subscriber('sniper_cam/image/compressed', CompressedImage, image_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    image_subscriber()
