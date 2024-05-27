#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray
from scipy.io import loadmat
import numpy as np

mat_data = loadmat('Models_5.mat')
AB1 = mat_data['AB1']
AB2 = mat_data['AB2']
AB3 = mat_data['AB3']
AB4 = mat_data['AB4']
AB5 = mat_data['AB5']

class ModelPublisher:
    def __init__(self):
        rospy.init_node('model_publisher', anonymous=True)
        
        # Subscriber to some topic, adjust 'topic_name' and message type as needed
        self.state_subscriber = rospy.Subscriber(
            '/koopman_state',  # Update 'input_topic' to your actual topic
            Float32MultiArray,          # Update this if you're subscribing to a different message type
            self.state_callback)
        
        self.point_subscriber = rospy.Subscriber(
            'setpoint',  # Update 'input_topic' to your actual topic
            PointStamped,          # Update this if you're subscribing to a different message type
            self.point_callback)
        
        # Publisher for sending the goal, adjust 'goal_topic' and message type as needed
        self.model_publisher = rospy.Publisher(
            'model_selected',  # Update 'goal_topic' to your actual goal topic name
            int,         # Update this if your goal is a different message type
            queue_size=10)

        #initialize the msg matrix
        self.ZandU = np.zeros((9 * 5 + 3, 1))

    def send_msg(self, msg):
        # This method publishes the msg
        self.goal_publisher.publish(msg)
        rospy.loginfo("Sent msg: %s", msg)

    def state_callback(self, msg):
        # This callback function is called whenever a message is received
        self.ZandU[:45] = msg
    
    def point_callback(self, msg):
        self.ZandU[45:] = msg
        AB = np.hstack((X_plus,)) @ np.linalg.pinv(self.ZandU)

def main():

    try:
        send_code = SendCode()
        rospy.spin(send_code)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
