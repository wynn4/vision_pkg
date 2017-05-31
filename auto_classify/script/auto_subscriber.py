#! /usr/bin/env python

## Simple ROS node that:
## -subscribes to state_image topic
## -runs image through convolution network
## -when airplane is in final descent we return best solution

## Jaron Ellingson (adapted from Jesse Wynn) AUVSI '17
## Note: As of 05/22/17 this node only works as designed with Ubuntu 14.04 and OpenCV version 2.4

import rospy
from fcu_common.msg import State
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
import sys, os
from scipy.misc import imsave, imshow
from PIL import Image
from auto_classify.srv import *
from sniper_cam.msg import stateImage

#import batch_utils
#import get_dir_images

class AutoSubscriber(object):

    def __init__(self):
        # setup state_image subscriber
        self.auto_image_subscriber = rospy.Subscriber('state_image', stateImage, self.auto_image_callback, queue_size=10)

        # setup landing subscriber
        #self.auto_landing_subscriber = rospy.Subscriber('asdfasdf', landing_something, self.final_results_callback, queue_size=10) 
        
        # initialize counter
        self.total = 0

        # initialize the image to save
        shape = 964, 1288, 3
        self.image_save = np.zeros(shape, np.uint8)

        # initialize categories
        self.alpha = alpha  = ['a','b','c','d','e','f','g','h','i','j',
                               'k','l','m','n','o','p','q','r','s','t',
                               'u','v','w','x','y','z',
                               '0','1','2','3','4','5','6','7','8','9']
        self.colors = ['brown', 'grey', 'red', 'green', 'blue', 'yellow', 'orange', 'white', 'black', 'purple']
        self.shapes =['star', 'cross', 'triangle', 'square', 'circle', 'halfcircle', 'rectangle', 'quarter_circle', 'trapezoid', 'pentagon', 'hexagon', 'heptagon', 'octagon']

        # initialize in memory results
        self.color_dirs = {}
        for c in self.colors:
            self.color_dirs[c] = {}
            self.color_dirs[c]['all_ret'] = []
            self.color_dirs[c]['orig_img'] = []

        # create a CvBridge object
        self.bridge = CvBridge()

    def softmax_client(self,img,loss,oh_sc_i,oh_lc_i,oh_s_i,oh_l_i):
        rospy.wait_for_service('softmax')
        try:
            softmax = rospy.ServiceProxy('softmax', Softmax)
            resp1 = softmax(img, loss,oh_sc_i,oh_lc_i,oh_s_i,oh_l_i)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def auto_image_callback(self, msg):
        # pull off the image portion of the message
        self.image_save = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        h,w,c= self.image_save.shape
        one =  self.image_save[0:h//2, 0:w//3] 
        two = self.image_save[0:h//2, w//3:2*w//3] 
        three = self.image_save[0:h//2, 2*w//3:w] 
        four = self.image_save[h//2:h, 0:w//3] 
        five = self.image_save[h//2:h, w//3:2*w//3] 
        six = self.image_save[h//2:h, 2*w//3:w]

        all_imgs = [one,two,three,four,five,six]
     
        # Resize images to fit into network and then get results   
        for i,image in enumerate(all_imgs):
            #cv2.imwrite(str(i+1)+'.png' ,cv2.resize(image, (224,224)))
            temp_image = self.bridge.cv2_to_imgmsg(cv2.resize(image, (224,224)), "bgr8")
            ret = self.softmax_client(temp_image,False,-1,-1,-1,-1)
            ch_idx = np.argmax(ret.sc) 
            if ret.sc[ch_idx] > .75:
                self.total = self.total + 1
                print('accepted: ' + str(self.total))
                self.color_dirs[self.colors[ch_idx]]['orig_img'].append(temp_image)
                self.color_dirs[self.colors[ch_idx]]['all_ret'].append(ret)
                if not os.path.exists('colors/'+ self.colors[ch_idx]):
                    os.makedirs('colors/'+self.colors[ch_idx])
                r_img = np.array(ret.return_img)
                r_img = r_img.reshape([24,24,3])               
                imsave('./colors/' + self.colors[ch_idx]+'/'+str(i)+'.png', r_img)
            else:
                place = 0
                #print('denied: ' + str(i))

    def final_results_callback(self, msg):
        for color in self.color_dirs:
        
            print('----------'+color+'--------------')
        
            all_ret = self.color_dirs[color]['all_ret']
            if len(all_ret) == 0:
                print('Not Found')
                continue


            sum_sc = []
            sum_lc = []
            for i in range(10):
                sum_sc.append(0)
                sum_lc.append(0)
        

            winning = []
            for i in range(4):
                winning.append([])
            for r in all_ret:
                #print r.sc
                winning[0].append(np.argmax(r.sc))
                winning[1].append(np.argmax(r.lc))
                winning[2].append(np.argmax(r.s))
                winning[3].append(np.argmax(r.l))
                for i in range(len(r.sc)):
                    sum_sc[i] = sum_sc[i] + r.sc[i]
                #for i in range(len(r.lc)):
                #    sum_lc[i] = sum_lc[i] + r.lc[i]
     
            #print(winning)
            #print(sum_sc)
        
            best_sc = np.argmax(sum_sc) 
            print("Best Shape Color", self.colors[best_sc])
        
            #best_lc = np.argmax(sum_lc) 
            #print("Best Letter Color", best_lc)
        
            w_sc_idx = -1
            w_sc = 0


            for i,ret in enumerate(all_ret):
                if ret.sc[best_sc] > w_sc:
                    w_sc = ret.sc[best_sc]
                    w_sc_idx = i

        
            print('Best Letter Color',
                   self.colors[np.argmax(all_ret[w_sc_idx].lc)])   
            print('Best Shape', 
                   self.shapes[np.argmax(all_ret[w_sc_idx].s)]) 
            print('Best Letter', 
                   self.alpha[np.argmax(all_ret[w_sc_idx].l)]) 
        
            if not os.path.exists('colors_winners/'+color):
                os.makedirs('colors_winners/'+color)
            r_img = np.array(all_ret[w_sc_idx].return_img)
            r_img = r_img.reshape([24,24,3])
            imsave('./' + 'colors_winners/'+color+'/'+str(i)+'.png', r_img)
            
        

def main():
    #initialize the node
    rospy.init_node('auto_subscriber')

    #create instance of class that subscribes to the stamped_image
    subscriber = AutoSubscriber()
    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()
    subscriber.final_results_callback(2)

if __name__ == '__main__':
    main()
