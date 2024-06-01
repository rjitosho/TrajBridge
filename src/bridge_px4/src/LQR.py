#!/usr/bin/env python3

import rospy
import numpy as np
import os 

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped

class LQRController:
    def __init__(self):
        rospy.init_node('lqr')
        
        # Publishers
        self.pos_pub = rospy.Publisher("gcs/setpoint/position",PointStamped,queue_size=1)
        self.att_pub = rospy.Publisher("gcs/setpoint/attitude",QuaternionStamped,queue_size=1)
        
        # Subscribers
        self.koopman_state_sub = rospy.Subscriber('/koopman_state', Float32MultiArray, self.koopman_state_callback)
        self.last_koopman_state = None

        self.dt = 0.05 # 20 Hz
        self.kf = 0 # counter
        self.pre_counter = 0
        self.pre_counter_max = 10
        self.post_counter = 0
        self.post_counter_max = 0

        # Load controller file contents
        self.feedback_multiplier = rospy.get_param("lqr/feedback_multiplier")
        traj_name = rospy.get_param("lqr/traj_name")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        address = dir_path+"/../controllers/"+traj_name
        data = np.load(address)

        # Extract the data
        self.x_nom = data['x_nom']
        self.u_nom = data['u_nom']
        self.K = data['K']
        self.T = self.u_nom.shape[1]

    def koopman_state_callback(self, msg):
        self.last_koopman_state = msg

    def traj_out(self, event=None):
        # Unpack some stuff
        kf = self.kf
        t_now = rospy.Time.now()

        # compute the control input
        u = np.array([0, 0, 1.5]) # hover

        if self.pre_counter < self.pre_counter_max:
            self.pre_counter += 1

        elif kf < self.T:
            control_adjustment = self.K @ (np.array(self.last_koopman_state.data) - self.x_nom[:, kf])
            u = self.u_nom[:, kf] + self.feedback_multiplier * control_adjustment
            self.kf += 1
            print('Nominal Control Input: {}'.format(self.u_nom[:, kf]))
            print('Control Adjustment: {}'.format(control_adjustment))
        
        elif self.post_counter < self.post_counter_max:
            u = self.u_nom[:, -1] # hold the last control input
            self.post_counter += 1

        else:
            print("------------- TRAJECTORY COMPLETE -------------")            
            rospy.signal_shutdown("TRAJECTORY COMPLETE")

        print('Control Input: {}'.format(u))

        # Variables to publish
        pos_msg = PointStamped()
        att_msg = QuaternionStamped()

        # Position
        pos_msg.header.stamp = t_now
        pos_msg.header.seq = kf
        pos_msg.header.frame_id = "map"

        pos_msg.point.x = u[0]
        pos_msg.point.y = u[1]
        pos_msg.point.z = u[2]

        # Attitude
        att_msg.header.stamp = t_now
        att_msg.header.seq = kf
        att_msg.header.frame_id = "map"
        
        att_msg.quaternion.w = 1
        att_msg.quaternion.x = 0
        att_msg.quaternion.y = 0
        att_msg.quaternion.z = 0

        # Publish
        self.pos_pub.publish(pos_msg)
        self.att_pub.publish(att_msg)


if __name__ == '__main__':
    lqr = LQRController()

    # wait for the first messages to arrive
    while lqr.last_koopman_state is None:
        rospy.sleep(lqr.dt)

    print("------------- STARTING LQR CONTROLLER --------------")
    rospy.Timer(rospy.Duration(lqr.dt), lqr.traj_out)
    rospy.spin()