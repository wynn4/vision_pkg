#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import time
from uav_msgs.msg import stampedImage
import sys
import tf

class imageStamper:
    def __init__(self,args):
        input_topic = rospy.get_param('~spotter_image')
        output_topic = rospy.get_param('~spotter_stamp')
        # self.imageSub = rospy.Subscriber(args[1],Image,self.callback)
        self.imageSub = rospy.Subscriber(input_topic,Image,self.callback)
        # self.iSPub = rospy.Publisher(args[2],stampedImage,queue_size=1)
        self.iSPub = rospy.Publisher(output_topic,stampedImage,queue_size=1)
        self.listener = tf.TransformListener()

        self.base_frame = rospy.get_param('~base_frame')
        self.MAV_frame = rospy.get_param('~UAS_frame')

        #when we have something publishing states, will need subscribers for that
        self.SI = stampedImage()

    def callback(self,data):

        image_time = data.header.stamp

        try:
            now = image_time
            # self.listener.waitForTransform("/turtle1", "/turtle2", now, rospy.Duration(1.0))
            # (trans, rot) = self.listener.lookupTransform("/turtle1", "/turtle2", now)
            self.listener.waitForTransform(self.base_frame, self.MAV_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(self.base_frame, self.MAV_frame, now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print('e')

        phi,theta,psi = tf.transformations.euler_from_quaternion(rot)

        pn,pe,pd = trans
        pd = -100

        alpha_az = 0.0*np.pi/180.0
        alpha_el = -45.0*np.pi/180.0

        self.SI.pn = pn
        self.SI.pe = pe
        self.SI.pd = pd
        self.SI.phi = phi
        self.SI.theta = theta
        self.SI.psi = psi
        self.SI.alpha_az = alpha_az
        self.SI.alpha_el = alpha_el
        self.SI.image = data

        self.iSPub.publish(self.SI)

def main(args):
    rospy.init_node('stamper',anonymous=True)
    imageSer=imageStamper(args)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: rosrun image_stamper image_stamper.py <input_image_topic> <output_stamped_topic>')
    else:
        main(sys.argv)
