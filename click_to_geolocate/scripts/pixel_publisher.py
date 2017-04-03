#! /usr/bin/env python

## Simple publisher that publishes pixel coordinates on mouse click
## Publishes to the 'pixel_data' topic
## Jesse Wynn AUVSI '17
## Camera clicker class provides way to create a camera object to get NED coordinates
## of a target from. based on Chapter 13 of Small Unmanned Aircraft by Beard and McLain
## Peter Schleede AUVSI '17

import rospy
import cv2
import numpy as np
import sys
import tf
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from click_to_geolocate.msg import FloatList
from fcu_common.msg import State

'''
class to perform calculations on camera image
'''
class camClick:

    '''
    initializes camera parameter
    Imputs:
        gimbal_pos:     3x1 list, [north,east,down].T, position of gimbal
                        center relative to MAV body center
        v:              2x1 float, [v_x,v_y].T, field of view of camera in degrees
    '''
    def __init__(self,gimbal_pos,v):

        #transform from gimbal center to MAV center (expressed in body frame)
        self.MAV_to_gimbal = np.array([[gimbal_pos[0]],[gimbal_pos[1]],[gimbal_pos[2]]])

        #set default gimbal angles. will be overwritten when getNED called
        self.alpha_az = 0.0
        self.alpha_el = -np.pi/4

        #set field of view
        self.v_w = v[0]*np.pi / 180.0
        self.v_h = v[1]*np.pi / 180.0

    '''
    Inputs:
        states_image:   6x1 list, [pn,pe,pd,phi,theta,psi].T, states of the
                        MAV (position and attitude only)
        gimbal_angles:  2x1 list, [alpha_az,alpha_el].T, gimbal position
                        angles
                        alpha_az is a right-handed roation about k^b
                        negative alpha_el points towards ground
    '''
    def setStatesandAngles(self, states_image, gimbal_angles):
        self.pn = states_image[0]
        self.pe = states_image[1]
        self.pd = states_image[2]
        self.phi = states_image[3]
        self.theta = states_image[4]
        self.psi = states_image[5]

        self.alpha_az = gimbal_angles[0]
        self.alpha_el = gimbal_angles[1]

    '''
    gets position of the object in the inertial frame
    Required Set Previously:
        self.states_image   [pn,pe,pd,phi,theta,psi]
        self.gimbal_angles  [alpha_az,alpha_el]
                            alpha_az is a right-handed rotation about k^b
                            negative alpha_el points towards ground
    Inputs:
        pixel_pt:       2x1 ndarray, [u,v].T, pixel location from origin in top
                        left corner
        image_size:     2x1 ndarray, [height,width].T, size of image
        R_b_i:          3x3 ndarray, rotation matrix from body to inertial frame
                        currently doesn't work if initialized in this method,
                        must be passed in
    '''
    def getNED(self,pixel_pt,image_size,R_b_i):
        position = np.array([[self.pn],[self.pe],[self.pd]])

        #used in transformation
        h = -self.pd
        k_i = np.array([[0],[0],[1]])

        #rotation matrices. nomenclature is R_[initial fram]_[final frame]
        #for some reason, if I create R_b_i here, the method doesn't work
        #passing it in works so pass it in

        R_g_b = np.array([[np.cos(self.alpha_el)*np.cos(self.alpha_az),np.cos(self.alpha_el)*np.sin(self.alpha_az),-np.sin(self.alpha_el)], \
                           [-np.sin(self.alpha_az),np.cos(self.alpha_az),0],[np.sin(self.alpha_el)*np.cos(self.alpha_az), \
                            np.sin(self.alpha_el)*np.sin(self.alpha_az),np.cos(self.alpha_el)]]).T

        R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

        #get image size
        height = image_size[0]
        width = image_size[1]

        pt_x = pixel_pt[0]
        pt_y = pixel_pt[1]

        #l-unit vector from camera towards click (assumes camera FOV is square wrt pixels)
        #look in literature for how to do this in a rectangular image

        M_w = width
        f_w = M_w / (2.0*np.tan(self.v_w/2.0))
        M_h = height
        f_h = M_h / (2.0*np.tan(self.v_h/2.0))

        #distances in pixels from origin of camera frame. origin is center of image.
        #don't forget that y is down in camera frame

        eps_x = pt_x - width / 2.0
        eps_y = pt_y - height / 2.0

        F_w = np.sqrt(f_w**2 + eps_x**2 + eps_y**2)
        F_h = np.sqrt(f_h**2 + eps_x**2 + eps_y**2)

        l_c_w = (1/F_w)*np.array([[eps_x],[eps_y],[f_w]])
        big_term_w = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c_w)))
        den_w = np.dot(k_i.T,big_term_w)
        l_c_h = (1/F_h)*np.array([[eps_x],[eps_y],[f_h]])
        big_term_h = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c_h)))
        den_h = np.dot(k_i.T,big_term_h)

        p_obj_w = position + R_b_i.dot(self.MAV_to_gimbal) + h*big_term_w/den_w
        p_obj_h = position + R_b_i.dot(self.MAV_to_gimbal) + h*big_term_h/den_h
        p_obj = [float(p_obj_h[0]),float(p_obj_w[1]),float(p_obj_w[2])]
        rospy.logwarn(p_obj)

        return p_obj

'''
ROS class for managing data
'''
class listen_and_locate:

    def __init__(self,gimbal_pos,v):
        # self.image_sub = rospy.Subscriber('/image_stamped',stampedImage,self.image_cb)
        self.state_sub = rospy.Subscriber('/state',State,self.state_cb)
        self.pub = rospy.Publisher('spotter_data', FloatList, queue_size=10)
        self.bridge = CvBridge()
        self.camera = camClick(gimbal_pos,v)

        self.pixPt = []
        self.refPt = FloatList()
        self.target_counter = 1.0

        #camera calibration parameters (this calibration could be re-done to be more accurate)
        #intrinsic parameters
        self.fx = 474.788647 #x focal length in pixels
        self.ox = 311.008804 #x coordinate of optical center in pixels
        self.fy = 467.476429 #y focal length in pixels
        self.oy = 212.330799 #y coordinate of optical center in pixels

        #distortion coefficients
        self.k1 = -4.67962e-01
        self.k2 = 2.92767e-01
        self.p1 = 1.810e-03
        self.p2 = 1.383e-03
        self.k3 = -1.19120e-01

        self.camMatrix = np.array([[self.fx, 0.0, self.ox],
                      [0.0, self.fy, self.oy],
                      [0.0, 0.0, 1.0]], dtype = np.float64)

        self.distCoeff = np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype = np.float64)

        self.image()

    '''
    receives the states and updates what the camera has
    '''
    def state_cb(self,data):
        states_image = [data.position[0],data.position[1],data.position[2],data.phi,data.theta,data.psi]
        gimbal_pos = [0.0*np.pi/180.0,-45.0*np.pi/180.0]

        self.camera.setStatesandAngles(states_image,gimbal_pos)

    def image(self):

        '''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data.image, "bgr8")
        except CvBridgeError as e:
            print(e)

        gimbal_pos = [data.alpha_az, data.alpha_el]
        states_image = [data.pn,data.pe,data.pd,data.phi,data.theta,data.psi]

        self.camera.setStatesandAngles(states_image,gimbal_pos)
        '''

        rate = rospy.Rate(30)

        #creates a named window for our camera, waits for mouse click
        cv2.namedWindow('spotter_cam')
        cv2.setMouseCallback('spotter_cam', self.click_and_pub_pixel_data)
        cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            ret, self.frame = cap.read()

            cv2.rectangle(self.frame, (80,80), (560,400), (0,0,255), 2)
            cv2.putText(self.frame,"Target: " + str(int(self.target_counter)),(5,20),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255))
            cv2.imshow('spotter_cam',self.frame)

            cv2.waitKey(1)
            rate.sleep()

        cap.release()
        cv2.destroyAllWindows()

    '''
    mouse click callback. if left mouse button, uses self.camera to publish
    an NED coordinate requires stampedImages to be subscribed to. if right
    mouse button, increments target counter
    '''
    def click_and_pub_pixel_data(self, event, x, y, flags, param):
        N_targets  = 9.0 #CHANGE THIS TO CORRECT NUMBER OF TARGETS (must be float)
        if event == cv2.EVENT_LBUTTONDOWN:

            if 80 <= x <= 560 and 80 <= y <= 400:
                src = np.array([[[x, y]]], dtype = np.float64)  #src is input pixel coordinates

                #undistortPoints() returns a 3D array of the projection of the pixel, to the image sensor
                undistortedPixel = cv2.undistortPoints(src,self.camMatrix,self.distCoeff)

                #multiply the projection by the focal length and then add the offset to convert back to pixels
                undistortedPixel1 = undistortedPixel[0][0][0]*self.fx + self.ox
                undistortedPixel2 = undistortedPixel[0][0][1]*self.fy + self.oy

                #the new undistorted pixel values
                x_new = undistortedPixel1
                y_new = undistortedPixel2

                self.pixPt = [x_new,y_new]

                #still annoyingly necessary
                phi = self.camera.phi
                theta = self.camera.theta
                psi = self.camera.psi

                R_b_i = np.array([[np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)], \
                                       [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi) \
                                        +np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],[np.cos(phi)*np.sin(theta)*np.cos(psi) \
                                        +np.sin(phi)*np.sin(psi),np.cos(phi)*np.sin(theta)*np.sin(psi) \
                                        -np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]]).T

                size = self.frame.shape
                image_size = [size[0],size[1]]

                refPt = self.camera.getNED(self.pixPt,image_size,R_b_i)
                refPt.append(self.target_counter)
                self.refPt.data = refPt

                self.pub.publish(self.refPt)

            else:
                pass

        elif event == cv2.EVENT_RBUTTONDOWN:

            self.target_counter += 1.0
            if self.target_counter == N_targets+1.0: #assumes 9 targets
                self.target_counter = 1.0

        elif event == cv2.EVENT_MBUTTONDOWN:
            self.target_counter -= 1.0
            if self.target_counter == 0.0:
                self.target_counter = N_targets

        else:
            pass

def main(args):
    rospy.init_node('locator', anonymous=True)

    #parameters for camera (eventually will be obtained on initialization)
    gimbal_pos = [0.5,0,0.1]
    v = [120.0,87.0]
    # v = [45.0,45.0]

    listen = listen_and_locate(gimbal_pos,v)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
