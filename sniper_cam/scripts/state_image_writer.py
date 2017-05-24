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
from fcu_common.msg import State
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

        # setup mouse click callback
        cv2.namedWindow('sniper cam image')
        cv2.setMouseCallback('sniper cam image', self.click_and_save)

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.alpha_az = 0.0
        self.alpha_el = 0.0

        # initialize image parameters
        self.img_width = 0.0
        self.img_height = 0.0
        self.fov_w = 47.2   #with PointGrey Chameleon3 and 6mm lens from M12lenses.com
        self.fov_h = 34.0   #with PointGrey Chameleon3 and 6mm lens from M12lenses.com

        # initialize target number
        self.target_number = 0

        self.status = "Standby..."
        self.time_str = "_"
        self.color = 0, 0, 255

        # initialize the image to save
        shape = 964, 1288, 3
        self.image_save = np.zeros(shape, np.uint8)

        # set vision_files directory
        self.image_directory = os.path.expanduser('~') + "/Desktop/vision_files/target_images/"

        self.txt_directory = os.path.expanduser('~') + "/Desktop/vision_files/"

        self.bridge = CvBridge()


    def state_image_callback(self, msg):
        # pull off the state info from the message
        self.pn = msg.pn
        self.pe = msg.pe
        self.pd = msg.pd

        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.chi % (2*math.pi)    # here we approximate psi as chi mod 2*pi

        self.alpha_az = msg.azimuth
        self.alpha_el = msg.elevation

        # direct conversion to CV2 of the image portion of the message
        #np_arr = np.fromstring(msg.image.data, dtype=np.uint8)
        #image_display = cv2.imdecode(np_arr, 1)
        #self.image_save = cv2.imdecode(np_arr, 1)

        # pull off the image portion of the message
        image_display = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        self.image_save = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

        # get the width and height of the image
        height, width, channels = image_display.shape
        self.img_width = width
        self.img_height = height

        # get the time
        self.get_current_time()

        # display the image
        cv2.rectangle(image_display,(0,0),(310,60),(0,0,0),-1)
        cv2.putText(image_display,"Status: ",(5,25),cv2.FONT_HERSHEY_PLAIN,2,(0,255,0))
        cv2.putText(image_display, self.status,(140,25),cv2.FONT_HERSHEY_PLAIN,2,(self.color))
        cv2.putText(image_display,"Date/Time: " + self.time_str,(5,50),cv2.FONT_HERSHEY_PLAIN,1,(197,155,19))
        cv2.imshow('sniper cam image', image_display)
        # wait about half a second
        cv2.waitKey(500)


    def click_and_save(self, event, x, y, flags, param):
        # if user clicks on target in the image frame
        if event == cv2.EVENT_LBUTTONDOWN and self.target_number > 0:
            #write the image to all_images folder
            self.write_image_to_file()

            #write the associated state data to filename
            self.write_state_to_file()
        else:
            pass




    def get_current_time(self):
        dt = datetime.now()
        m_time = dt.microsecond
        m_time = str(m_time)[:3]
        time_now = strftime("%m%d%y-%H:%M:%S:" + m_time, localtime())
        self.time_str = str(time_now)


    def write_image_to_file(self):
        target_folder = "target_" + str(self.target_number) + "/"
        filename = self.time_str + ".jpg"
        cv2.imwrite(self.image_directory + target_folder + filename, self.image_save)


    def write_state_to_file(self, location):
        filename = "state_data_all_images.txt"
        f = open(self.txt_directory + filename, 'a')
        try:
            f.write(self.time_str + "," + str(self.pn) + "," + str(self.pe) + "," str(self.pd) + ","
                    str(self.phi) + "," str(self.theta) + "," str(self.psi) + "," str(self.alpha_az) + ","
                    str(self.alpha_el))
            f.write("\n")
        finally:
            f.close()


def main():
    #initialize the node
    rospy.init_node('sniper_geo_locator')

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
