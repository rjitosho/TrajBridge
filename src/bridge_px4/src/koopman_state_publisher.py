#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class StatePublisher:
    def __init__(self):
        rospy.init_node('state_publisher')
        
        self.state_pub = rospy.Publisher('/koopman_state', Float32MultiArray, queue_size=10)
        
        self.drone_pose_sub = rospy.Subscriber('/drone5/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        # self.drone_pose_sub = rospy.Subscriber('/drone5/mavros/vision_pose/pose', PoseStamped, self.drone_pose_callback)
        self.tip_pose_sub = rospy.Subscriber('/vrpn_client_node/tip/pose', PoseStamped, self.tip_pose_callback)
        
        self.last_drone_pose = None
        self.last_tip_pose = None

        self.last_drone_time = None
        self.last_tip_time = None
        
        self.state_buffer = []
        self.rate = rospy.Rate(20)  # Publish at 20 Hz
        
    def drone_pose_callback(self, msg):
        self.last_drone_pose = msg

    def tip_pose_callback(self, msg):
        self.last_tip_pose = msg

    def get_current_configuration(self):
        if self.last_drone_time is not None or self.last_tip_time is not None:
            time_since_last_drone = (self.last_drone_pose.header.stamp - self.last_drone_time).to_sec()
            time_since_last_tip = (self.last_tip_pose.header.stamp - self.last_tip_time).to_sec()
            if time_since_last_drone > 0.1 or time_since_last_tip > 0.1:
                rospy.logwarn('Drone or tip pose is not being updated. Time since last drone pose: {}, time since last tip pose: {}'.format(time_since_last_drone, time_since_last_tip))
            self.last_drone_time = self.last_drone_pose.header.stamp
            self.last_tip_time = self.last_tip_pose.header.stamp

        [x, y, z] = [self.last_drone_pose.pose.position.x, self.last_drone_pose.pose.position.y, self.last_drone_pose.pose.position.z]
        [qx, qy, qz] = [self.last_drone_pose.pose.orientation.x, self.last_drone_pose.pose.orientation.y, self.last_drone_pose.pose.orientation.z]
        [x_tip, y_tip, z_tip] = [self.last_tip_pose.pose.position.x, self.last_tip_pose.pose.position.y, self.last_tip_pose.pose.position.z]
        return [x, y, z, qx, qy, qz, x_tip, y_tip, z_tip]

    def run(self):
        # wait for the first messages to arrive
        while self.last_drone_pose is None or self.last_tip_pose is None:
            self.rate.sleep()

        # fill the buffer to start
        for _ in range(5):
            self.state_buffer = self.get_current_configuration() + self.state_buffer
            self.rate.sleep()

        # Publish the state at 20 Hz
        while not rospy.is_shutdown():           
            # Update the state buffer
            self.state_buffer = self.get_current_configuration() + self.state_buffer[:-9]
            
            # Publish the state
            self.state_pub.publish(Float32MultiArray(data=self.state_buffer))

            # Sleep
            self.rate.sleep()

if __name__ == '__main__':
    state_publisher = StatePublisher()
    state_publisher.run()
