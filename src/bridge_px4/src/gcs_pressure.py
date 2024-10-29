#!/usr/bin/env python

import rospy
from std_msgs.msg import String

AUTOMATED_CMD = False

def main():
    rospy.init_node('teensy_communication_pressure')
    pub = rospy.Publisher('pressure_cmd', String, queue_size=1)

    pressure_cmd = -1
    
    while not rospy.is_shutdown():
        if AUTOMATED_CMD:
            pub.publish("-.1")
            # pub.publish(str(round(pressure_cmd,1)))
            pressure_cmd += 0.02
            if pressure_cmd > 0.0:
                pressure_cmd = -.2
            rospy.sleep(1.1)

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
