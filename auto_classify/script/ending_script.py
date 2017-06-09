#!/usr/bin/env python
# license removed for brevity
import rospy
from auto_classify.msg import autoEnding
import os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class TestPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('auto_ending', autoEnding, queue_size=10)
        self.msg = autoEnding()
    
    def end_sequ(self):
        self.msg.end = True
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('ending_script', anonymous=True)
    try:
        test = TestPublisher()
        rospy.sleep(2.)
        test.end_sequ()
    except rospy.ROSInterruptException:
        print 'error'
        pass
