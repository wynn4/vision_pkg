#! /usr/bin/env python

## Simple ROS node that:
## -subscribes to state_image topic
## -displays the image portion of the message
## -if user clicks in the image window
##  -write image to all_targets folder
##  -write state data to all_state_data.txt

## Jesse Wynn AUVSI '17
## Note: As of 05/22/17 this node only works as designed with Ubuntu 14.04 and OpenCV version 2.4

import rospy
from sniper_cam.msg import stateImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
import os
import time
from time import strftime, localtime
from datetime import datetime


class StateImageWriter(object):

    def __init__(self):
        # setup state_image subscriber
        self.state_image_subscriber = rospy.Subscriber('state_image', stateImage, self.state_image_callback, queue_size=1)

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.chi = 0.0

        self.alpha_az = 0.0
        self.alpha_el = 0.0

        # initialize time_str
        self.time_str = "_"

        # initialize the image to save
        shape = 964, 1288, 3
        self.image_save = np.zeros(shape, np.uint8)

        # set vision_files directories
        self.image_directory = os.path.expanduser('~') + "/Desktop/vision_files/all_images/"

        self.txt_directory = os.path.expanduser('~') + "/Desktop/vision_files/all_state_files/"

        # create a CvBridge object
        self.bridge = CvBridge()

        # initialize counter
        self.counter = 0

    def state_image_callback(self, msg):
        # pull off the state info from the message
        self.pn = msg.pn
        self.pe = msg.pe
        self.pd = msg.pd

        self.phi = msg.phi
        self.theta = msg.theta
        self.chi = msg.chi

        self.alpha_az = msg.azimuth
        self.alpha_el = msg.elevation

        # pull off the image portion of the message
        # image_display = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        self.image_save = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

        # increment the counter
        self.counter += 1

        # get the time
        self.get_current_time()

        # # display the image
        # cv2.rectangle(image_display,(0,0),(1288,20),(0,0,0),-1)
        # cv2.putText(image_display,"Date/Time: " + self.time_str,(980,15),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0))
        # cv2.putText(image_display, "Saved Images: " + str(self.counter),(575,15),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255))
        # cv2.imshow('sniper cam state_image writer', image_display)
        # #wait about half a second
        # cv2.waitKey(500)

        #write the image to all_images folder
        self.write_image_to_file()

        #write the associated state data to filename
        self.write_state_to_file()


    def get_current_time(self):
        dt = datetime.now()
        m_time = dt.microsecond
        m_time = str(m_time)[:3]
        time_now = strftime("%m%d%y_%H-%M-%S-" + m_time, localtime())
        self.time_str = str(time_now)


    def write_image_to_file(self):
        filename = self.time_str + ".jpg"
        cv2.imwrite(self.image_directory + filename, self.image_save)

        print "Images written: " + str(self.counter)


    def write_state_to_file(self):
        filename = self.time_str + ".txt"
        f = open(self.txt_directory + filename, 'w')
        try:
            f.write(str(self.pn) + "," + str(self.pe) + "," + str(self.pd) + ","
            + str(self.phi) + "," + str(self.theta) + "," + str(self.chi) + ","
            + str(self.alpha_az) + "," + str(self.alpha_el))
            f.write("\n")
        finally:
            f.close()


def main():
    #initialize the node
    rospy.init_node('state_image_writer')

    #create instance of class that subscribes to the stamped_image
    subscriber = StateImageWriter()
    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
