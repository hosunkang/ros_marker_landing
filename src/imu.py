#!/usr/bin/python
import serial 
import math
import os
import serial.tools.list_ports_linux as sp
import rospy
from rospy.exceptions import ROSInterruptException
from rospy.topics import Publisher
from std_msgs.msg import String

list = sp.comports()
connected= []
for i in list:
    PORTAPPLY = 'sudo chmod 666 ' + i.device
    os.system(PORTAPPLY)
    connected.append(i.device)

def de2ra(de):
    return de*math.pi/180

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)

class imuaction:
    def __init__(self):
        self.pub = rospy.Publisher('imudata', String, queue_size=30)
        self.sub = rospy.Subscriber('eulerdata', String, self.euler_caller)
    
    def talker(self):
        while not rospy.is_shutdown():
            datas = ser.readline()
            self.pub.publish(datas)

    def euler_caller(self, data):
        datas = str(data)
        euler = datas.split('"')[1]
        ser.write(euler)

if __name__ == '__main__':
    imu = imuaction()
    rospy.init_node('imu', anonymous=True)
    imu.talker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass