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
        self.drone_pub = rospy.Publisher("/drone5/mavros/vision_pose/pose",PoseStamped,queue_size=1)

        # Subscribers
        rospy.Subscriber("/gcs/setpoint/position", PointStamped, self.gcs_setpoint_position_cb)
        self.gcs_setpoint = PointStamped()
        self.gcs_setpoint.point.x = 0
        self.gcs_setpoint.point.y = 0
        self.gcs_setpoint.point.z = 1.5
        
        # flying vine state
        self.z = np.zeros(45)
        self.z[2::9] = 1.5
        self.z[7::9] = .5

    def gcs_setpoint_position_cb(self, data):
        self.gcs_setpoint = data
    
    def fake_pose_out(self, event=None):
        # Publish fake drone pose
        drone_msg = PoseStamped()
        drone_msg.header.stamp = rospy.Time.now()
        drone_msg.header.frame_id = "map"
        drone_msg.pose.position.x = self.gcs_setpoint.point.x
        drone_msg.pose.position.y = self.gcs_setpoint.point.y
        drone_msg.pose.position.z = self.gcs_setpoint.point.z
        self.drone_pub.publish(drone_msg)
        
        # Publish fake tip pose
        tip_msg = PoseStamped()
        tip_msg.header.stamp = rospy.Time.now()
        tip_msg.header.frame_id = "map"
        tip_msg.pose.position.x = self.gcs_setpoint.point.x
        tip_msg.pose.position.y = self.gcs_setpoint.point.y
        tip_msg.pose.position.z = self.gcs_setpoint.point.z-1
        self.tip_pub.publish(tip_msg)

if __name__ == '__main__':
    rospy.init_node('fake_flying_vine_node',disable_signals=True)
    gcs = GCS()
    gcs.fake_pose_out()

    rospy.Timer(rospy.Duration(gcs.dt), gcs.fake_pose_out)
    rospy.spin()

