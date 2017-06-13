#!/usr/bin/env python
# license removed for brevity
import rospy
from sniper_cam.msg import stateImage
import os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import random

class TestPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('state_image', stateImage, queue_size=50)
        self.msg = stateImage()
        self.bridge = CvBridge()
    
    def create_and_send(self, one_file):
        temp = cv2.imread('./test_images/'+one_file)
        self.msg.image = self.bridge.cv2_to_imgmsg(np.array(temp), "bgr8")
        self.msg.pn = random.uniform(0.0, 1.0)
        self.msg.pe = random.uniform(0.0, 1.0)
        self.msg.pd = random.uniform(0.0, 1.0)
        self.msg.phi = random.uniform(0.0, 1.0)
        self.msg.theta = random.uniform(0.0, 1.0)
        self.msg.chi = random.uniform(0.0, 1.0)
        self.msg.azimuth = random.uniform(0.0, 1.0)
        self.msg.elevation = random.uniform(0.0, 1.0)
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('test_publisher', anonymous=True)
    try:
        test = TestPublisher()
        filelist = os.listdir('./test_images')
        
        for i, one_file in enumerate(filelist):
            print i
            test.create_and_send(one_file)
            rospy.sleep(1.)
    except rospy.ROSInterruptException:
        pass
    rospy.sleep(1.)



