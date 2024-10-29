#!/usr/bin/env python

import rospy
from std_msgs.msg import String

INCREASING_PRESSURE = True

def main():
    rospy.init_node('teensy_communication_pressure')
    pub = rospy.Publisher('pressure_cmd', String, queue_size=1)

    pressure_cmd = -1
    
    while not rospy.is_shutdown():
        if INCREASING_PRESSURE:
            pub.publish(str(pressure_cmd) + "\n")
            pressure_cmd += 0.01
            rospy.sleep(0.5)

        else:
            input_str = input("Enter body pressure: ")
            try:
                _ = float(input_str)
            except ValueError:
                continue
            
            pub.publish(input_str)
            rospy.sleep(0.1)        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
