#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from click_to_geolocate.msg import FloatList
import numpy as np

class location_listener:
    def __init__(self):
        self.sub = rospy.Subscriber('/spotter_data', FloatList, self.callback)
        self.N_targets = 9       #CHANGE THIS TO CORRECT NUMBER OF TARGETS
        self.locations = np.zeros((self.N_targets,3))
        self.target_counter = np.zeros(self.N_targets)      #keeps track of how many times each target has been updated (for averaging)

        #target location publishers
        self.targ_pub = rospy.Publisher('pos_t1',FloatList,queue_size=10)

        self.listofpublishers = [0]*self.N_targets
        for i in range(self.N_targets):
            self.listofpublishers[i] = rospy.Publisher('pos_'+str(i+1),FloatList,queue_size=10)

        self.publish_main()

    #assume data comes in as [pn,pe,pd,index] where index is target number (1-based index)
    def callback(self,data):
        # earth_radius_utah = 6370000
        # lat_home = 39.9785960
        # lon_home = -112.0037920
        pn = data.data[0]
        pe = data.data[1]
        i = int(data.data[3]) - 1
        print(i)

        tot = (self.target_counter[i]*self.locations[i] + data.data[0:3]) / (self.target_counter[i]+1)
        self.target_counter[i] += 1
        self.locations[i] = tot

        # print('location: ' + str(data.data[0:3]))
        # print(self.locations)

        # msg = FloatList()
        # msg.data = self.locations[i]
        # self.listofpublishers[i].publish(msg)

        # lat = (pn/earth_radius_utah)*180/np.pi + lat_home
        # lon = (pe/(earth_radius_utah*np.cos(lat_home*np.pi/180)))*180/np.pi + lon_home

        # print data.data

        # print('Latitude = ' + str(lat))
        # print('Longitude = ' + str(lon))
        #rospy.loginfo(data)

    #continuously publishes the incoming (averaged) data
    def publish_main(self):
        r = rospy.Rate(5)       #5 Hz

        while not rospy.is_shutdown():
            for i in range(self.N_targets):
                msg = FloatList()
                msg.data = self.locations[i]
                self.listofpublishers[i].publish(msg)

            r.sleep()


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    l = location_listener()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
