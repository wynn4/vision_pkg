#! /usr/bin/env python

## Simple ROS node that:
## -cycles through the all_images directory
## -displays the image
## -if user clicks target in image:
##  -capture x,y pixel coord
##  -calculate target NED location
##  -wirte image to target-sorted file
##  -write location and heading to target-sorted file
## -right click in image window increments target number
## -middle click decrements target number

import rospy
from sensor_msgs.msg import CompressedImage
from sniper_cam.msg import stateImage
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

        # get the earth's radius from current location
        self.R_earth = rospy.get_param('~radius_earth', 6370027)    #default set to radius at Webster Field MD

        # initialize home location (from gps_init)
        self.home = [40.174500, -111.651665, 0.0] # lat, lon, alt (default BYU)

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
        self.fov_w = math.radians(47.2)   # field of view width with PointGrey Chameleon3 and 6mm lens from M12lenses.com
        self.fov_h = math.radians(34.0)   # field of view height with PointGrey Chameleon3 and 6mm lens from M12lenses.com

        # set the camera calibration parameters (hard-coded from calibration)
        self.fx = 1622.655818
        self.cx = 617.719411
        self.fy = 1624.083744
        self.cy = 407.261222
        self.k1 = -0.596969
        self.k2 = 0.339409
        self.p1 = 0.000552
        self.p2 = -0.000657
        self.k3 = 0.000000

        self.cameraMatrix = np.array([[self.fx, 0.0, self.cx],
                                [0.0, self.fy, self.cy],
                                [0.0, 0.0, 1.0]], dtype = np.float64)

        self.distCoeff = np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype = np.float64)

        # initialize target number
        self.target_number = 0

        self.status = "Standby..."

        self.color = 0, 0, 255

        # initialize the image to save
        shape = 964, 1288, 3
        self.image_save = np.zeros(shape, np.uint8)

        # initialize the image ID
        self.image_id = "_"

        self.got_gps_init = False


    def chapter_13_geolocation(self,x,y,msg):
        self.pn = msg.pn
        self.pe = msg.pe
        self.pd = msg.pd

        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.chi % (2*math.pi) 

        self.alpha_az = msg.azimuth
        self.alpha_el = msg.elevation

        # get the undistorted pixel coordinate
        src = np.array([[[x, y]]], dtype = np.float64)  #src is input pixel coordinates
        undistortedProjection = cv2.undistortPoints(src,self.cameraMatrix,self.distCoeff)

        # multiply the projection by the focal length and then add the offset to convert back to pixels
        undistortedPixel_x = undistortedProjection[0][0][0]*self.fx + self.cx
        undistortedPixel_y = undistortedProjection[0][0][1]*self.fy + self.cy

        # the new undistorted pixel values
        x_new = undistortedPixel_x
        y_new = undistortedPixel_y

        # geolocate the object (target) using technique from UAV book chapter 13

        # capture pixel coordinates
        px = x_new;
        py = y_new;

        # position of the UAV
        p_uav = np.array([[self.pn],[self.pe],[self.pd]])

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

        # calculate the location of the target in NED
        p_obj_w = p_uav + h*big_term_w/den_w                        #EQ 13.18
        p_obj_h = p_uav + h*big_term_h/den_h
        p_obj = [float(p_obj_h[0]),float(p_obj_w[1]),float(p_obj_w[2])]

        #print "Target " + str(self.target_number) + " image and location captured"
        #print p_obj
        # print eps_x, eps_y
        # print "\n"


        # convert location from NED to lat lon
        lat = self.home[0] + math.degrees(math.asin(p_obj[0]/self.R_earth))
        lon = self.home[1] + math.degrees(math.asin(p_obj[1]/(math.cos(math.radians(self.home[0]))*self.R_earth)))
	#return [2,4]
        return [lat, lon]
        


    def gps_init_cb(self, gps_init_array):
        self.home[0] = gps_init_array.data[0]
        self.home[1] = gps_init_array.data[1]
        self.home[2] = gps_init_array.data[2]

        self.got_gps_init = True

        #print "Home location received"
        #print "Home Lat: " + str(self.home[0])
        #print "Home Lon: " + str(self.home[1])
        #print "Home Alt: " + str(self.home[2])

        self.gps_init_sub.unregister()


def main():
    #initialize the node
    rospy.init_node('auto_geo_locator')

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
