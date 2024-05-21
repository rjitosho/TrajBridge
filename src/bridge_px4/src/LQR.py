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

        # Load controller file contents
        self.feedback_multiplier = rospy.get_param("lqr/feedback_multiplier")
        traj_name = rospy.get_param("lqr/traj_name")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        address = dir_path+"/../controllers/"+traj_name
        data = np.load(address)

        # Extend the nominal state and control arrays
        n, m = 10, 10
        self.x_nom = self.extend_array(data['x_nom'], n, m)
        self.u_nom = self.extend_array(data['u_nom'], n, m)
        self.T = self.u_nom.shape[1]

        # Load the controller gains
        self.K = data['K']

    def extend_array(self, X, n, m):
        """
        Extend an array X by adding n columns of the first column of X at the start
        and m columns of the last column of X at the end.
        
        Parameters:
        X (np.ndarray): The input array.
        n (int): Number of columns to add at the start.
        m (int): Number of columns to add at the end.
        
        Returns:
        np.ndarray: The extended array.
        """
        # Get the shape of X
        rows, cols = X.shape

        # Create the new array with the desired shape
        extended = np.zeros((rows, cols + n + m))

        # Fill the first n columns with the first column of X
        extended[:, :n] = X[:, 0].reshape(rows, 1)

        # Fill the middle part with X
        extended[:, n:n+cols] = X

        # Fill the last m columns with the last column of X
        extended[:, -m:] = X[:, -1].reshape(rows, 1)

        return extended

    def koopman_state_callback(self, msg):
        self.last_koopman_state = msg

    def traj_out(self, event=None):
        # Unpack some stuff
        kf = self.kf
        t_now = rospy.Time.now()

        # compute the control input
        control_adjustment = self.K @ (np.array(self.last_koopman_state.data) - self.x_nom[:, kf])
        u = self.u_nom[:, kf] + self.feedback_multiplier * control_adjustment
        
        print('Nominal Control Input: {}'.format(self.u_nom[:, kf]))
        print('Control Adjustment: {}'.format(control_adjustment))

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

        # Update Counter
        self.kf += 1

        if self.kf == self.T:
            print("------------- TRAJECTORY COMPLETE -------------")
            rospy.signal_shutdown("TRAJECTORY COMPLETE")

if __name__ == '__main__':
    lqr = LQRController()

    # wait for the first messages to arrive
    while lqr.last_koopman_state is None:
        rospy.sleep(lqr.dt)

    print("------------- STARTING LQR CONTROLLER --------------")
    rospy.Timer(rospy.Duration(lqr.dt), lqr.traj_out)
    rospy.spin()
