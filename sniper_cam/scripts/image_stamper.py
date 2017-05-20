#! /usr/bin/env python

## simple ROS node that:
## -subscribes to CompressedImage from PointGrey Camera
## -pairs the CompressedImage with the relevant state data
## -pubishes a state-stamped image at 1 hz
## Jesse Wynn AUVSI '17


import rospy
from sensor_msgs.msg import CompressedImage
from fcu_common.msg import State
from sniper_cam.msg import stateImage
import cv2
import math
import numpy as np

class ImageStamper(object):
    # simple class that takes in images and adds the current state data to them with a custom message type

    def __init__(self):
        self.image_subscriber = rospy.Subscriber('image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.state_subscriber = rospy.Subscriber('/state', State, self.state_callback, queue_size=10)
        self.state_image_publisher = rospy.Publisher('state_image', stateImage, queue_size=1)

        # get the incoming frame rate from the camera
        self.frame_rate = rospy.get_param('~frame_rate_in', 15)

        # create stateImage object
        self.state_image_msg = stateImage()

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = -5.0

        self.phi = 0.0
        self.theta = 0.0
        self.chi = 0.0

        self.alpha_az = math.radians(90)
        self.alpha_el = math.radians(-45)

        # initialize counter
        self.counter = 0


    def image_callback(self, msg):
        # increment the counter
        self.counter += 1

        # publish at 1 Hz
        if self.counter == self.frame_rate:
            # reset counter
            self.counter = 0

            # fill out the stateImage message
            self.state_image_msg.pn = self.pn
            self.state_image_msg.pe = self.pe
            self.state_image_msg.pd = self.pd
            self.state_image_msg.phi = self.phi
            self.state_image_msg.theta = self.theta
            self.state_image_msg.chi = self.chi
            self.state_image_msg.azimuth = self.alpha_az
            self.state_image_msg.elevation = self.alpha_el
            self.state_image_msg.image = msg

            # publish the message
            self.state_image_publisher.publish(self.state_image_msg)
        else:
            pass


    def state_callback(self, data):
        self.pn = data.position[0]
        self.pe = data.position[1]
        self.pd = data.position[2]

        self.phi = data.phi
        self.theta = data.theta
        self.chi = data.chi


def main():
    # initialize the node
    rospy.init_node('image_stamper')

    # create instance of class ImageStamper
    subscriber = ImageStamper()
    # spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    # OpenCV cleanup
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
