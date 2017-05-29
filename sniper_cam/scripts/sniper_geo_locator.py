#! /usr/bin/env python

## Simple ROS node that:
## -cycles through the all_images directory
## -displays the image
## -if user clicks target in image:
##  -capture x,y pixel coord
##  -calculate target NED location
##  -wirte image to target-sorted file
##  -write NED location and heading to target-sorted file
## -right click in image window increments target number
## -middle click decrements target number

import rospy
from sensor_msgs.msg import CompressedImage
from sniper_cam.msg import stateImage
from fcu_common.msg import State
from std_msgs.msg import Float64, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
import os
import glob
import time
from time import strftime, localtime
from datetime import datetime


class SniperGeoLocator(object):
    # class that takes care of geolocation of ground targets based on Small Unmanned Aircraft Theory and Practice Chapter 13
    # assumes 'flat_earth'

    def __init__(self):
        #setup gps_init subscriber
        self.gps_init_sub = rospy.Subscriber('/gps_init', Float32MultiArray, self.gps_init_cb)

        # setup mouse click callback
        self.window = 'sniper cam image'
        cv2.namedWindow(self.window)
        cv2.setMouseCallback(self.window, self.click_and_locate)

        # home location (from gps_init)
        self.home = [0.0, 0.0, 0.0] # lat, lon, alt

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
        #self.time_str = "_"
        self.color = 0, 0, 255

        # initialize the image to save
        shape = 964, 1288, 3
        self.image_save = np.zeros(shape, np.uint8)

        # set vision_files directory
        self.image_path = os.path.expanduser('~') + "/Desktop/vision_files/all_images/*.jpg"

        self.file_list = glob.glob(self.image_path)

        # sort the files to be in chronological order
        self.file_list.sort(key=os.path.getmtime)


        self.image_number = 0


        self.image_directory = os.path.expanduser('~') + "/Desktop/vision_files/target_images_sorted/"

        self.txt_directory = os.path.expanduser('~') + "/Desktop/vision_files/target_locations_sorted/"

        self.all_txt_directory = os.path.expanduser('~') + "/Desktop/vision_files/all_state_files/"

        self.image_id = "_"



    def display_image(self):
        # pull off the state info from the message
        #self.pn = msg.pn
        #self.pe = msg.pe
        #self.pd = msg.pd

        #self.phi = msg.phi
        #self.theta = msg.theta
        #self.psi = msg.chi % (2*math.pi)    # here we approximate psi as chi mod 2*pi

        #self.alpha_az = msg.azimuth
        #self.alpha_el = msg.elevation

        # direct conversion to CV2 of the image portion of the message
        #np_arr = np.fromstring(msg.image.data, dtype=np.uint8)
        #image_display = cv2.imdecode(np_arr, 1)
        #self.image_save = cv2.imdecode(np_arr, 1)

        # pull off the image portion of the message
        #image_display = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        #self.image_save = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")


        # read in the image
        filename = self.file_list[self.image_number]
        stuff = filename.strip().split('/')
        image_name = stuff[-1]  #get the last part of the file path

        #set the current image id
        self.image_id = image_name[:19] #get the first 19 characters of the image name (everything except '.jpg')
        image_display = cv2.imread(os.path.expanduser('~') + "/Desktop/vision_files/all_images/" + image_name)
        self.image_save = cv2.imread(os.path.expanduser('~') + "/Desktop/vision_files/all_images/" + image_name)

        # get the width and height of the image
        height, width, channels = image_display.shape
        self.img_width = width
        self.img_height = height

        # get the time
        #self.get_current_time()

        # draw the interface on the display image
        cv2.rectangle(image_display,(0,0),(310,60),(0,0,0),-1)
        cv2.putText(image_display,"Status: ",(5,25),cv2.FONT_HERSHEY_PLAIN,2,(0,255,0))
        cv2.putText(image_display, self.status,(140,25),cv2.FONT_HERSHEY_PLAIN,2,(self.color))
        cv2.putText(image_display,"Date/Time: " + self.image_id,(5,50),cv2.FONT_HERSHEY_PLAIN,1,(197,155,19))
        cv2.rectangle(image_display,(width-180,0),(width,15),(0,0,0),-1)
        cv2.putText(image_display, "Image number: " + str(self.image_number),(width-175,12),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0))

        # display the image
        cv2.imshow(self.window, image_display)
        key = cv2.waitKey(500)
        #print key

        if key == 32 and len(self.file_list) > self.image_number + 1:   # spacebar
            self.image_number += 1
            self.update_file_list()
        elif key == 98 and self.image_number > 0: # 'B' key for 'back'
            self.image_number -= 1
        elif key == 32 and len(self.file_list) == self.image_number + 1:
            print "End of file list reached"
            self.update_file_list()
        else:
            pass


    def click_and_locate(self, event, x, y, flags, param):
        # if user clicks on target in the image frame
        if event == cv2.EVENT_LBUTTONDOWN and self.target_number > 0  and self.image_number > 0:
            #parse text file containing associated state data
            self.parse_state_file()
            self.chapter_13_geolocation(x,y)

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.target_number += 1
            self.status = "Target " + str(self.target_number)
            self.color = 0, 255, 0

        elif event == cv2.EVENT_MBUTTONDOWN and self.target_number > 0:
            if self.target_number == 1:
                self.target_number = 0
                self.status = "Standby..."
                self.color = 0, 0, 255
            else:
                self.target_number -= 1
                self.status = "Target " + str(self.target_number)
        else:
            pass



    def chapter_13_geolocation(self,x,y):
        # geolocate the object (target) using technique from UAV book chapter 13

        # position of the UAV
        p_uav = np.array([[self.pn],[self.pe],[self.pd]])

        #capture pixel coordinates
        px = x;
        py = y;

        # convert to pixel locations measured from image center (0,0)
        eps_x = px - self.img_width/2.0
        eps_y = py - self.img_height/2.0

        # define rotation from body to inertial frame (R_b_i = R_i_b transpose)
        R_b_i = np.array([[np.cos(self.theta)*np.cos(self.psi),np.cos(self.theta)*np.sin(self.psi),-np.sin(self.theta)], \
                               [np.sin(self.phi)*np.sin(self.theta)*np.cos(self.psi)-np.cos(self.phi)*np.sin(self.psi),np.sin(self.phi)*np.sin(self.theta)*np.sin(self.psi) \
                                +np.cos(self.phi)*np.cos(self.psi),np.sin(self.phi)*np.cos(self.theta)],[np.cos(self.phi)*np.sin(self.theta)*np.cos(self.psi) \
                                +np.sin(self.phi)*np.sin(self.psi),np.cos(self.phi)*np.sin(self.theta)*np.sin(self.psi) \
                                -np.sin(self.phi)*np.cos(self.psi), np.cos(self.phi)*np.cos(self.theta)]]).T

        # define rotation from gimbal frame to body frame (R_g_b = R_b_g transpose)
        R_g_b = np.array([[np.cos(self.alpha_el)*np.cos(self.alpha_az),np.cos(self.alpha_el)*np.sin(self.alpha_az),-np.sin(self.alpha_el)], \
                       [-np.sin(self.alpha_az),np.cos(self.alpha_az),0],[np.sin(self.alpha_el)*np.cos(self.alpha_az), \
                        np.sin(self.alpha_el)*np.sin(self.alpha_az),np.cos(self.alpha_el)]]).T

        # define rotation from camera to gimbal frame (R_c_g = R_g_c transpose)
        R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

        # define vector k_i as unit vector aligned with inertial down axis
        k_i = np.array([[0],[0],[1]])

        # define height above ground h, as -pd
        h = -self.pd

        # now we need to define vector el_hat_c (here we break it into el_hat_c_w and el_hat_c_h because camera frame is rectangular instead of square)
        f_w = self.img_width/(2.0*np.tan(self.fov_w/2.0))               #EQ 13.5
        f_h = self.img_height/(2.0*np.tan(self.fov_h/2.0))

        F_w = np.sqrt(f_w**2 + eps_x**2 + eps_y**2)                 #EQ 13.6
        F_h = np.sqrt(f_h**2 + eps_x**2 + eps_y**2)

        el_hat_c_w = (1/F_w)*np.array([[eps_x],[eps_y],[f_w]])      #EQ 13.9
        el_hat_c_h = (1/F_h)*np.array([[eps_x],[eps_y],[f_h]])

        big_term_w = R_b_i.dot(R_g_b.dot(R_c_g.dot(el_hat_c_w)))
        den_w = np.dot(k_i.T,big_term_w)

        big_term_h = R_b_i.dot(R_g_b.dot(R_c_g.dot(el_hat_c_h)))
        den_h = np.dot(k_i.T,big_term_h)

        # calculate the location of the target
        p_obj_w = p_uav + h*big_term_w/den_w                        #EQ 13.18
        p_obj_h = p_uav + h*big_term_h/den_h
        p_obj = [float(p_obj_h[0]),float(p_obj_w[1]),float(p_obj_w[2])]

        print p_obj
        print eps_x, eps_y

        # write the image to file
        self.write_image_to_file()

        # write the location data to file
        self.write_location_to_file(p_obj)


    # def get_current_time(self):
    #     dt = datetime.now()
    #     m_time = dt.microsecond
    #     m_time = str(m_time)[:3]
    #     time_now = strftime("%m%d%y-%H:%M:%S:" + m_time, localtime())
    #     self.time_str = str(time_now)


    def parse_state_file(self):
        filename = self.image_id + ".txt"
        f = open(self.all_txt_directory + filename, 'r')
        for line in f:
            #get a list of the state data
            state_data = line.strip().split(',')

            #update the member variables for geolocation
            self.pn = float(state_data[0])
            self.pe = float(state_data[1])
            self.pd = float(state_data[2])

            self.phi = float(state_data[3])
            self.theta = float(state_data[4])
            self.psi = float(state_data[5])%(2*math.pi) #here we approximate psi as chi mod 2*pi

            self.alpha_az = float(state_data[6])
            self.alpha_el = float(state_data[7])


    def write_image_to_file(self):
        target_folder = "target_" + str(self.target_number) + "/"
        filename = self.image_id + ".jpg"
        cv2.imwrite(self.image_directory + target_folder + filename, self.image_save)


    def write_location_to_file(self, location):
        filename = "target_" + str(self.target_number) + "_locations.txt"
        f = open(self.txt_directory + filename, 'a')
        try:
            f.write(self.image_id + "," + str(self.target_number) + "," +
                    str(location[0]) + "," + str(location[1]) + "," + str(math.degrees(self.psi)))
            f.write("\n")
        finally:
            f.close()

    def gps_init_cb(self, gps_init_array):
        self.home[0] = gps_init_array.data[0]
        self.home[1] = gps_init_array.data[1]
        self.home[2] = gps_init_array.data[2]

        self.gps_init_sub.unregister()

    def update_file_list(self):
        self.file_list = glob.glob(self.image_path)
        self.file_list.sort(key=os.path.getmtime)



def main():
    #initialize the node
    rospy.init_node('sniper_geo_locator')

    #create instance of SniperGeoLocator class
    locator = SniperGeoLocator()

    while not rospy.is_shutdown():
        locator.display_image()

    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
