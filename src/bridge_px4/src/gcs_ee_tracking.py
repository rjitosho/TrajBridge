#!/usr/bin/env python3

import rospy
from typing import Tuple
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PoseStamped
import os 

class GCS:
    def __init__(self):
        # Input Params
        traj_name = rospy.get_param("gcs/traj_name")
        hold = rospy.get_param("gcs/hold")
        laps = rospy.get_param("gcs/laps")

        # Trajectory Variables
        self.T,self.X,self.aN,self.dt = self.traj_gen(traj_name,hold,laps)

        # Counters
        self.kf = 0
        self.kN = 0

        # Publishers
        self.pos_pub = rospy.Publisher("gcs/setpoint/position",PointStamped,queue_size=1)
        self.att_pub = rospy.Publisher("gcs/setpoint/attitude",QuaternionStamped,queue_size=1)

        # Subscribers
        rospy.Subscriber("/vrpn_client_node/tip/pose", PoseStamped, self.tip_pose_cb)
        self.tip_pose = PoseStamped()
        rospy.Subscriber("/drone5/mavros/vision_pose/pose", PoseStamped, self.vision_pose_cb)
        self.vision_pose = PoseStamped()

        # History of last 10 tip poses
        self.tip_pose_hist = []

        # Wait for first tip pose
        while self.tip_pose.pose.position.x == 0:
            pass

        # fill up history
        for i in range(10):
            self.tip_pose_hist.append([self.tip_pose.pose.position.x,
                                       self.tip_pose.pose.position.y,
                                       self.tip_pose.pose.position.z])
            rospy.sleep(self.dt)

    def tip_pose_cb(self, data):
        self.tip_pose = data
        # print("tip_pose_cb: " + str(self.tip_pose.pose.position.x) + ", " + str(self.tip_pose.pose.position.y) + ", " + str(self.tip_pose.pose.position.z))
    
    def traj_gen(self,traj_name,hold,laps) -> Tuple[np.ndarray,np.ndarray]:
        # Get Address of Trajectory
        dir_path = os.path.dirname(os.path.realpath(__file__))
        address = dir_path+"/../trajectories/"+traj_name
        
        # Extract Data
        data = np.genfromtxt(address,delimiter=',')
        Td:np.ndarray = data[0,:]
        Xd:np.ndarray = data[1:14,:]

        # Unpack Some Stuff
        dt = Td[1]-Td[0]

        # Generate Hold
        hold = dt*(hold//dt)
        Nh = int(hold/dt)+1
        Xh = np.tile(Xd[:,0].reshape((13,1)),(1,Nh))

        # Generate State Trajectory
        X = np.hstack((Xh,np.tile(Xd,(1,laps))))

        # Compute some useful stuff
        Nd = Xd.shape[1]
        aN = np.zeros(laps+1,dtype=int)           # array of milestones
        aN[0] = int(Nh)
        for i in range(1,laps+1):
            aN[i] = aN[i-1]+Nd
        
        T = np.arange(0,(aN[-1]+1)*dt,dt)

        return T,X,aN,dt
            
    def traj_out(self, event=None):
        # Unpack some stuff
        kf = self.kf
        t_now = rospy.Time.now()

        # Variables to publish
        pos_msg = PointStamped()
        att_msg = QuaternionStamped()

        # P control on tip position
        x_correction = self.X[0,kf] - self.tip_pose.pose.position.x
        y_correction = self.X[0,kf] - self.tip_pose.pose.position.y
        z_correction = self.X[0,kf] - self.tip_pose.pose.position.z
        vine_length = 1.5

        # clip corrections
        max_correction = 0.0
        x_correction = np.clip(x_correction, -max_correction, max_correction)
        y_correction = np.clip(y_correction, -max_correction, max_correction)
        z_correction = np.clip(z_correction, -max_correction, max_correction)

        # Position
        pos_msg.header.stamp = t_now
        pos_msg.header.seq = kf
        pos_msg.header.frame_id = "map"

        pos_msg.point.x = self.X[0,kf] + x_correction
        pos_msg.point.y = self.X[1,kf] + y_correction
        pos_msg.point.z = self.X[2,kf] + z_correction + vine_length

        # Attitude
        att_msg.header.stamp = t_now
        att_msg.header.seq = kf
        att_msg.header.frame_id = "map"
        
        att_msg.quaternion.w = self.X[6,kf]
        att_msg.quaternion.x = self.X[7,kf]
        att_msg.quaternion.y = self.X[8,kf]
        att_msg.quaternion.z = self.X[9,kf]

        # Publish
        self.pos_pub.publish(pos_msg)
        self.att_pub.publish(att_msg)

        # Update Counter
        self.kf += 1

        if self.kf >= self.aN[self.kN]:
            if self.kN == 0:
                print("[GCS]: Completed Loiter. Proceeding to Trajectory")
            else:
                print("[GCS]: Completed Lap:",self.kN,"of",len(self.aN)-1)
            
            self.kN += 1

        if self.kf == self.aN[-1]:
            print("[GCS]: Mission Complete")
            rospy.signal_shutdown("GCS Send Complete")

if __name__ == '__main__':
    rospy.init_node('gcs_node',disable_signals=True)
    gcs = GCS()

    rospy.Timer(rospy.Duration(gcs.dt), gcs.traj_out)
    rospy.spin()

