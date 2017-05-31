#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
from beginner_tutorials.srv import *
import rospy

import tensorflow as tf
import numpy as np
from spatial_transformer import transformer
sess = tf.InteractiveSession()
import cv2
from cv_bridge import CvBridge
from scipy.misc import imsave


bridge = CvBridge()

# placeholders
xs = tf.placeholder(tf.float32, shape=[None, 224*224*3], name="images")
let_ = tf.placeholder(tf.float32, shape=[None, 36])
sha_ = tf.placeholder(tf.float32, shape=[None, 13])
let_col_ = tf.placeholder(tf.float32, shape=[None, 10])
sha_col_ = tf.placeholder(tf.float32, shape=[None, 10])
keep_prob = tf.placeholder(tf.float32)

# initialization functions
def weight_variable(shape):
  initial = tf.truncated_normal(shape, stddev=0.2)
  return tf.Variable(initial)

def bias_variable(shape):
  initial = tf.constant(0.1, shape=shape)
  return tf.Variable(initial)

# layer functions
def conv2d(x, W):
  return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
  return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

x_image = tf.reshape(xs, [-1,224,224,3])
# vgg = vgg16.vgg16( x_image, 'vgg16_weights.npz', sess )
#
# layers = [ 'conv5_1','conv5_2' ]
# ops = [ getattr( vgg, x ) for x in layers ]

with tf.variable_scope('transformer_network') as trans_scope:
    W_conv1 = weight_variable([5, 5, 3, 32])
    b_conv1 = bias_variable([32])
    h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
    h_pool1 = max_pool_2x2(h_conv1)

    W_conv2 = weight_variable([5, 5, 32, 32])
    b_conv2 = bias_variable([32])
    h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
    h_pool2 = max_pool_2x2(h_conv2)

    # W_conv3 = weight_variable([5, 5, 32, 32])
    # b_conv3 = bias_variable([32])
    # h_conv3 = tf.nn.relu(conv2d(h_pool2, W_conv3) + b_conv3)
    # h_pool3 = max_pool_2x2(h_conv3)

    W_fc1 = weight_variable([56*56*32, 1024])
    b_fc1 = bias_variable([1024])
    h_pool2_flat = tf.reshape(h_pool2, [-1, 56*56*32])
    # h_pool3_flat = tf.reshape(h_pool3, [-1, 128*72*32])
    h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

    h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

    initial = np.array([0, 0, 112])
    initial = initial.astype('float32')
    initial = initial.flatten()

    W_fc2 = tf.Variable(tf.truncated_normal([1024,3], stddev=1e-8))# tf.zeros([1024, 3]) # weight_variable([1024, 3])
    b_fc2 = tf.Variable(initial_value=initial)
    y_conv=tf.matmul(h_fc1_drop, W_fc2) + b_fc2

x_est, y_est, scale_out = tf.split(axis=1, num_or_size_splits=3, value=y_conv)
scale_est = tf.abs(scale_out)*0.0803/112
trans = tf.maximum(tf.minimum(tf.concat([scale_est*tf.ones_like(x_est), tf.zeros_like(x_est), x_est/112,  tf.zeros_like(x_est), scale_est*tf.ones_like(x_est), y_est/112], 1), 1.0),-1.0)
x_trans = transformer(x_image, trans, (24,24))

'''with tf.variable_scope('rotator_network') as rotate_scope:
    W_conv1 = weight_variable([5, 5, 3, 32])
    b_conv1 = bias_variable([32])
    h_conv1 = tf.nn.relu(conv2d(x_trans, W_conv1) + b_conv1)
    h_pool1 = max_pool_2x2(h_conv1)

    W_conv2 = weight_variable([5, 5, 32, 32])
    b_conv2 = bias_variable([32])
    h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
    h_pool2 = max_pool_2x2(h_conv2)

    W_fc1 = weight_variable([6*6*32, 1024])
    b_fc1 = bias_variable([1024])
    h_pool2_flat = tf.reshape(h_pool2, [-1, 6*6*32])
    h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

    h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

    initial = np.array([1, 0, 0, 1])
    initial = initial.astype('float32')
    initial = initial.flatten()

    W_fc2 = tf.Variable(tf.truncated_normal([1024,4], stddev=1e-8))# tf.zeros([1024, 3]) # weight_variable([1024, 3])
    b_fc2 = tf.Variable(initial_value=initial)
    r_conv=tf.matmul(h_fc1_drop, W_fc2) + b_fc2

r_00, r_01, r_10, r_11 = tf.split(axis=1, num_or_size_splits=4, value=r_conv)
rot_trans = tf.maximum(tf.minimum(tf.concat([scale_est*r_00, scale_est*r_01, x_est/112,  scale_est*r_10, scale_est*r_11, y_est/112], 1), 1.0),-1.0)
x_rot_trans = transformer(x_image, rot_trans, (24,24))
'''
with tf.variable_scope('classifyer_network') as class_scope:
    W_conv1 = weight_variable([5, 5, 3, 32])
    b_conv1 = bias_variable([32])
    h_conv1 = tf.nn.relu(conv2d(x_trans, W_conv1) + b_conv1)
    h_pool1 = max_pool_2x2(h_conv1)

    W_conv2 = weight_variable([5, 5, 32, 64])
    b_conv2 = bias_variable([64])
    h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
    h_pool2 = max_pool_2x2(h_conv2)

    W_fc1 = weight_variable([6 * 6 * 64, 1024])
    b_fc1 = bias_variable([1024])
    h_pool2_flat = tf.reshape(h_pool2, [-1, 6 * 6 * 64])
    h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

    # keep_prob = tf.placeholder(tf.float32)
    h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

    with tf.variable_scope("letter"):
        W_fc2 = tf.get_variable( "W", [1024, 36], tf.float32,
                                  tf.random_normal_initializer( stddev=np.sqrt(2 / np.prod(h_fc1_drop.get_shape().as_list()[1:])) ) ) # weight_variable([1024, 36])
        b_fc2 = bias_variable([36])
        let_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

    with tf.variable_scope("shape"):
        W_fc2 = tf.get_variable( "W", [1024, 13], tf.float32,
                                  tf.random_normal_initializer( stddev=np.sqrt(2 / np.prod(h_fc1_drop.get_shape().as_list()[1:])) ) ) # weight_variable([1024, 36])
        b_fc2 = bias_variable([13])
        sha_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

    with tf.variable_scope("letter_color"):
        W_fc2 = tf.get_variable( "W", [1024, 10], tf.float32,
                                  tf.random_normal_initializer( stddev=np.sqrt(2 / np.prod(h_fc1_drop.get_shape().as_list()[1:])) ) ) # weight_variable([1024, 36])
        b_fc2 = bias_variable([10])
        let_col_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

    with tf.variable_scope("shape_color"):
        W_fc2 = tf.get_variable( "W", [1024, 10], tf.float32,
                                  tf.random_normal_initializer( stddev=np.sqrt(2 / np.prod(h_fc1_drop.get_shape().as_list()[1:])) ) ) # weight_variable([1024, 36])
        b_fc2 = bias_variable([10])
        sha_col_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

with tf.name_scope('Cost'):
        cross_entropies = tf.reduce_mean(-tf.reduce_sum(let_ * tf.log(tf.clip_by_value(let_conv,1e-10,1)), axis=[1])) + \
                    .5 * tf.reduce_mean(-tf.reduce_sum(sha_ * tf.log(tf.clip_by_value(sha_conv,1e-10,1)), axis=[1])) + \
                    .25 * tf.reduce_mean(-tf.reduce_sum(let_col_ * tf.log(tf.clip_by_value(let_col_conv,1e-10,1)), axis=[1])) + \
                    .05 * tf.reduce_mean(-tf.reduce_sum(sha_col_ * tf.log(tf.clip_by_value(sha_col_conv,1e-10,1)), axis=[1]))
        loss = cross_entropies #+ 0.1*tf.reduce_mean(tf.abs(tf.matrix_determinant(tf.reshape(r_conv,[-1,2,2])) - 1 ))

saver = tf.train.Saver()
saver.restore(sess, "script/tf_logs_2/whole_model.ckpt")
print("Whole model restored")

brdg = CvBridge()

def handle_softmax(req):
    print "Returning Image"
    cv_image = brdg.imgmsg_to_cv2(req.image, "rgb8")
    img = np.asarray(cv_image)
    imgs = np.zeros([1,224*224*3])
    imgs[0,:] = img.reshape((1,224*224*3))
    r_loss = -1.0
    if req.loss: 

        rxt = np.zeros(1728, dtype=np.float32)

        oh_l = np.zeros(36)
        l  = np.zeros(36, dtype=np.float32)
        
        oh_s = np.zeros(13)
        s = np.zeros(13, dtype=np.float32)
        
        oh_lc = np.zeros(10)
        lc = np.zeros(10, dtype=np.float32)
        
        oh_sc = np.zeros(10)
        sc = np.zeros(10, dtype=np.float32)
        
        oh_l.put(req.oh_l_i,1)
        oh_s.put(req.oh_s_i,1)
        oh_lc.put(req.oh_lc_i,1)
        oh_sc.put(req.oh_sc_i,1)

        r_loss,rxt = sess.run([loss,x_trans],feed_dict={
            xs:imgs, 
            let_: [oh_l], 
            sha_: [oh_s], 
            let_col_: [oh_lc], 
            sha_col_: [oh_sc], 
            keep_prob:1.0
        })
        return SoftmaxResponse(rxt[0,:].flatten(), sc, lc, s, l, r_loss)
    else:
        rxt,l,s,lc,sc = sess.run([x_trans, let_conv, sha_conv,let_col_conv,sha_col_conv],feed_dict={
            xs:imgs, 
            keep_prob: 1.0})
        
        return SoftmaxResponse(rxt[0,:].flatten(), sc[0,:], lc[0,:], s[0,:], l[0,:], r_loss)

def softmax_server():
    rospy.init_node('softmax_server')
    s = rospy.Service('softmax', Softmax, handle_softmax)
    print "Ready to recieve Image"
    rospy.spin()

if __name__ == "__main__":
    softmax_server()
