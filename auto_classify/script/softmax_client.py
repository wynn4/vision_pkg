#!/usr/bin/env python

import sys, os
import rospy
from auto_classify.srv import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import batch_utils
import numpy as np 
from scipy.misc import imsave, imshow
from PIL import Image
from random import randint
import get_dir_images


bridge = CvBridge()

alpha  = ['a','b','c','d','e','f','g','h','i','j',
                 'k','l','m','n','o','p','q','r','s','t',
                          'u','v','w','x','y','z',
                                   '0','1','2','3','4','5','6','7','8','9']

colors = ['brown', 'grey', 'red', 'green', 'blue', 'yellow', 'orange', 'white', 'black', 'purple']
shapes = ['star','cross','triangle','square','circle','halfcircle','rectangle','quarter_circle', 'trapezoid', 'pentagon','hexagon', 'heptagon', 'octagon']

color_dirs = {}
for c in colors:
    color_dirs[c] = {}
    color_dirs[c]['all_ret'] = []
    color_dirs[c]['orig_img'] = []

def softmax_client(img,loss,oh_sc_i,oh_lc_i,oh_s_i,oh_l_i):
    rospy.wait_for_service('softmax')
    try:
        softmax = rospy.ServiceProxy('softmax', Softmax)
        resp1 = softmax(img, loss,oh_sc_i,oh_lc_i,oh_s_i,oh_l_i)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def try_loss():
    for color in color_dirs:
        
        print('----------'+color+'--------------')
        
        all_ret = color_dirs[color]['all_ret']
        if len(all_ret) == 0:
            print('Not Found')
            continue
        
        all_loss = []
        
        oh_sc_i = -1
        oh_lc_i = -1
        oh_s_i = -1
        oh_l_i = -1


        for i, image in enumerate(color_dirs[color]['orig_img']):
            o_ret = color_dirs[color]['all_ret'][i]
            oh_sc_i = np.argmax(o_ret.sc)
            oh_lc_i = np.argmax(o_ret.lc)
            oh_s_i = np.argmax(o_ret.s)
            oh_l_i = np.argmax(o_ret.l)
            ret = softmax_client(image, True,oh_sc_i,oh_lc_i,oh_s_i,oh_l_i)
            all_loss.append(ret.r_loss)
        min_idx = np.argmin(all_loss)
       
       
        print('Best Shape Color' , 
              colors[np.argmax(all_ret[min_idx].sc)]) 
        print('Best Letter Color',
              colors[np.argmax(all_ret[min_idx].lc)])   
        print('Best Shape', 
              shapes[np.argmax(all_ret[min_idx].s)]) 
        print('Best Letter', 
              alpha[np.argmax(all_ret[min_idx].l)]) 

        if not os.path.exists('colors_loss/'+color):
            os.makedirs('colors_loss/'+color)
        r_img = np.array(all_ret[min_idx].return_img)
        r_img = r_img.reshape([24,24,3])
        imsave('./' + 'colors_loss/'+color+'/'+str(i)+'.png', r_img)
        


if __name__ == "__main__":

    batch = []
    batch.append(get_dir_images.get_files()) 
    '''batch = batch_utils.next_batch(10)
    
    for i in range(10):
        temp = batch_utils.next_batch(10)
        for out in temp[0]:
            batch[0].append(out)
    for i in range(100):
        grass_dir = './grass/'
        num = randint(1,84)
        pre = ''
        if num <= 9:
            pre = '00'
        else:
            pre = '0'

        background = Image.open(grass_dir + pre + str(num) + '.png')
        b_w, b_h = background.size

        delta_x = randint(0,b_w-224)
        delta_y = randint(0,b_h-224)
        background = background.crop((delta_x, delta_y, delta_x + 224, delta_y + 224))
        batch[0].append(background)
    
    #all_ret = []
    '''
    import time

    start =  time.time()
    
    for i, img in enumerate(batch[0]):      
        #orig_image = img 
        #img.show()
        #img.save('./tf_logs/a_' + str(i) + '.png')
        img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR) 
	img = bridge.cv2_to_imgmsg(img, "bgr8")
    
        print "Requesting Image Data"
        ret = softmax_client(img, False, -1,-1,-1,-1)      
        ch_idx = np.argmax(ret.sc) 
        if ret.sc[ch_idx] > .75:
            color_dirs[colors[ch_idx]]['orig_img'].append(img)
            #all_ret.append(ret)
            if not os.path.exists('colors/'+ colors[ch_idx]):
                os.makedirs('colors/'+colors[ch_idx])
            r_img = np.array(ret.return_img)
            r_img = r_img.reshape([24,24,3])
            color_dirs[colors[ch_idx]]['all_ret'].append(ret)
            #print('./' + colors[ch_idx]+'/'+str(i)+'.png')
            imsave('./colors/' + colors[ch_idx]+'/'+str(i)+'.png', r_img)
        else:
            print('denied: ' + str(i))
            #all_ret.append(ret)
            #all_ret[i].sc =  [0,0,0,0,0,0,0,0,0,0]
        r_img = ret.return_img
        r_img = np.array(r_img)
        
        r_img = r_img.reshape([24,24,3])
        #imsave('./tf_logs/b_' + str(i) + '.png',r_img)
    
    #try_loss()

    for color in color_dirs:
        
        print('----------'+color+'--------------')
        
        all_ret = color_dirs[color]['all_ret']
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
        print("Best Shape Color", colors[best_sc])
        
        #best_lc = np.argmax(sum_lc) 
        #print("Best Letter Color", best_lc)
        
        w_sc_idx = -1
        w_sc = 0


        for i,ret in enumerate(all_ret):
            if ret.sc[best_sc] > w_sc:
                w_sc = ret.sc[best_sc]
                w_sc_idx = i

        
        print('Best Letter Color',
              colors[np.argmax(all_ret[w_sc_idx].lc)])   
        print('Best Shape' , shapes[np.argmax(all_ret[w_sc_idx].s)]) 
        print('Best Letter' , alpha[np.argmax(all_ret[w_sc_idx].l)]) 
        
        if not os.path.exists('colors_loss/'+color):
            os.makedirs('colors_loss/'+color)
        r_img = np.array(all_ret[w_sc_idx].return_img)
        r_img = r_img.reshape([24,24,3])
        imsave('./' + 'colors_loss/'+color+'/'+str(i)+'.png', r_img)
        end = time.time()
        print('Time Elapsed', end-start)
