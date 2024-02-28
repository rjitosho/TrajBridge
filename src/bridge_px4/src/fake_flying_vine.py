#!/usr/bin/env python3

import rospy
from typing import Tuple
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os 

class GCS:
    def __init__(self):
        self.dt = 0.05

        # Publishers
        self.tip_pub = rospy.Publisher("/vrpn_client_node/tip/pose",PoseStamped,queue_size=1)
        self.drone_pub = rospy.Publisher("/drone5/mavros/local_position/pose",PoseStamped,queue_size=1)

        # Subscribers
        rospy.Subscriber("/gcs/setpoint/position2", PointStamped, self.gcs_setpoint_position_cb)
        
        # fake data
        self.i = 2
        self.fake_data = np.load("/home/oem/flyingSysID/2024-01-11-12-29-27_EE-fig8-10s-motionplan_resampled_0-05.npz", allow_pickle=True)

    def gcs_setpoint_position_cb(self, data):
        self.i += 1
    
    def fake_pose_out(self, event=None):
        # Publish fake drone pose
        drone_msg = PoseStamped()
        drone_msg.header.stamp = rospy.Time.now()
        drone_msg.header.frame_id = "map"
        drone_msg.pose.position.x = self.fake_data["local_position"][self.i, 1]
        drone_msg.pose.position.y = self.fake_data["local_position"][self.i, 2]
        drone_msg.pose.position.z = self.fake_data["local_position"][self.i, 3]
        drone_msg.pose.orientation.w = self.fake_data["local_position"][self.i, 4]
        drone_msg.pose.orientation.x = self.fake_data["local_position"][self.i, 5]
        drone_msg.pose.orientation.y = self.fake_data["local_position"][self.i, 6]
        drone_msg.pose.orientation.z = self.fake_data["local_position"][self.i, 7]
        self.drone_pub.publish(drone_msg)
        
        # Publish fake tip pose
        tip_msg = PoseStamped()
        tip_msg.header.stamp = rospy.Time.now()
        tip_msg.header.frame_id = "map"
        tip_msg.pose.position.x = self.fake_data["tip_pose"][self.i, 1]
        tip_msg.pose.position.y = self.fake_data["tip_pose"][self.i, 2]
        tip_msg.pose.position.z = self.fake_data["tip_pose"][self.i, 3]
        self.tip_pub.publish(tip_msg)

if __name__ == '__main__':
    rospy.init_node('fake_flying_vine_node',disable_signals=True)
    gcs = GCS()
    gcs.fake_pose_out()

    rospy.Timer(rospy.Duration(gcs.dt), gcs.fake_pose_out)
    rospy.spin()

