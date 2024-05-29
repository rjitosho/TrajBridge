#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('teensy_communication_pressure')
    pub = rospy.Publisher('pressure_cmd', String, queue_size=1)

    while not rospy.is_shutdown():
        # OPTION 1: request body pressure
        input_str = input("Enter body pressure: ")
        try:
            _ = float(input_str)
        except ValueError:
            continue
        
        pub.publish(input_str)
        rospy.sleep(0.1)

        # OPTION 2: fixed body pressure
        # pub.publish("0.01")
        # rospy.sleep(1.0)
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
