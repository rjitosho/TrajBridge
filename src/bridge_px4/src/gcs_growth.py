# #!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('teensy_communication_python')
    pub = rospy.Publisher('growth_cmd', String, queue_size=1)

    while not rospy.is_shutdown():
        # request body pressure
        input_str = input("Enter body pressure: ")
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
