#!/usr/bin/env python
# license removed for brevity
import rospy
from sniper_cam.msg import stateImage
import os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class TestPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('state_image', stateImage, queue_size=50)
        self.msg = stateImage()
        self.bridge = CvBridge()
    
    def create_and_send(self, one_file):
        temp = cv2.imread('./test_images/'+one_file)
        self.msg.image = self.bridge.cv2_to_imgmsg(np.array(temp), "rgb8")
        self.msg.pn = 0.0
        self.msg.pe = 0.0
        self.msg.pd = 0.0
        self.msg.phi = 0.0
        self.msg.theta = 0.0
        self.msg.chi = 0.0
        self.msg.azimuth = 0.0
        self.msg.elevation = 0.0
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
