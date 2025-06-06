#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry

import json
import matplotlib.pyplot as plt

import os

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/hk_data" 

def cb_fn(data: Odometry, cb_args):
    odom = data
    pos = [
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            #Offset stored in the linear twist 
            odom.twist.twist.linear.x, 
            odom.twist.twist.linear.y,
            #True position stored in the angular twist
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.pose.pose.orientation.x

    ]

    t_start = cb_args[0]
    index_end = cb_args[1]
    sub_pt = cb_args[2][0]
    affected_pos_list = cb_args[3]

    t_now = rospy.get_rostime()
    elapsed = (t_now - t_start).to_sec()

    affected_pos_list.append(pos)

    # index = int(elapsed*10 % index_end)
    # if index >= index_end - 1:
    #     sub_pt.unregister()
    #     print("unregistered...")

    

def plot(pos):
    pos = np.array(pos)
    plt.figure()
    plt.plot(pos[:,0], pos[:,1], ".")
    plt.xlabel("lon")
    plt.ylabel("lat")
    plt.show()

def main():
    rospy.init_node("record_aff_pos")
    t_start = rospy.get_rostime()
    elapsed = 0
    index_end = 786
    sub_pt = []

    affected_pos_list = []
    cb_args = [t_start, index_end, sub_pt, affected_pos_list]
    odom_sub = rospy.Subscriber("/multipath/hb1/aff_odom", Odometry, callback=cb_fn, callback_args=cb_args)
    sub_pt.append(odom_sub)

    try:
        rospy.spin()
        with open(data_dir+"_affect_pos.json", "w") as fs:
            affected_pos_list = np.array(affected_pos_list)
            data = {"aff_x": affected_pos_list[:,0].tolist(),
                "aff_y": affected_pos_list[:,1].tolist(), 
                "offset_x": affected_pos_list[:,2].tolist(),  
                "offset_y": affected_pos_list[:,3].tolist(),
                "true_x": affected_pos_list[:,4].tolist(),
                "true_y": affected_pos_list[:,5].tolist(), 
                "numSV": affected_pos_list[:,6].tolist(),
                }
            json.dump(data, fs)

    except KeyboardInterrupt:
        print("shutting down...")

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("ros interrupt")
    