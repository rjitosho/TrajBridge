#!/usr/bin/env python

import rospy
from std_msgs.msg import String

CONTINUOUS_GROWTH = True
GROWTH_RATE = "7"

def main():
    rospy.init_node('teensy_communication_growth')
    pub = rospy.Publisher('growth_cmd', String, queue_size=1)

    while not rospy.is_shutdown():
        if CONTINUOUS_GROWTH:
            pub.publish(GROWTH_RATE)
            rospy.sleep(0.75)

        else:
            # request growth rate
            input_str = input("Enter growth rate: ")
            try:
                _ = float(input_str)
            except ValueError:
                rospy.loginfo("Invalid input")
                continue
            pub.publish(input_str)
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
