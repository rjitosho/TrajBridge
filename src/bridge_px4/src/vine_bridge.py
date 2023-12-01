#!/home/oem/virtualenvs/flyingvine/bin/python3
import rospy
from std_msgs.msg import String
import time
import serial

class VineBridge:
    def __init__(self):
        self.arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.05)

    # def write_read(self, x):
    #     self.arduino.write(bytes(x, 'utf-8'))
    #     # return [0,0]
    #     data = self.arduino.readline()
    #     offset = 14
    #     return_size = 2
    #     # TODO: fix temporary test
    #     return [float(data[0:-2]), 0.0]
    #     return [float(data[offset*i:(offset*i+12)]) for i in range(return_size)]

    def growth_cmd_cb(self, data):
        self.arduino.write(bytes('g'+data.data, 'utf-8'))

    def pressure_cmd_cb(self, data):
        self.arduino.write(bytes('p'+data.data, 'utf-8'))
    
    def listener(self):
        rospy.init_node('vine_bridge', anonymous=True)

        rospy.Subscriber("growth_cmd", String, self.growth_cmd_cb)
        rospy.Subscriber("pressure_cmd", String, self.pressure_cmd_cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    vb = VineBridge()
    vb.listener()