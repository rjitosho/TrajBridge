#!/usr/bin/env python3

import rospy
from typing import Tuple
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import scipy.io

import os 

class GCS:
    def __init__(self):
        self.dt = 0.05

        # Publishers
        self.z_pub = rospy.Publisher("/z_flying_vine",Float32MultiArray,queue_size=1)

        # Subscribers
        rospy.Subscriber("/gcs/setpoint/position2", PointStamped, self.gcs_setpoint_position_cb)
        
        # fake data
        self.i = 1
        mat_file = '/home/oem/flyingSysID/fig8_sim_real_history_size_5.mat'  
        mat_data = scipy.io.loadmat(mat_file)
        self.Z_all = mat_data['Z_all']

    def gcs_setpoint_position_cb(self, data):
        self.i += 1
    
    def fake_pose_out(self, event=None):
        self.z_pub.publish(Float32MultiArray(data = self.Z_all[:, self.i]))

if __name__ == '__main__':
    rospy.init_node('fake_flying_vine_node',disable_signals=True)
    gcs = GCS()
    gcs.fake_pose_out()

    rospy.Timer(rospy.Duration(gcs.dt), gcs.fake_pose_out)
    rospy.spin()

