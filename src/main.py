#!/usr/bin/python
import math
import os
import pickle
import sys
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import serial
import serial.tools.list_ports_linux as sp
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from mathtool import mathtool

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

class markerLanding:
    def __init__(self):
        self.mt = mathtool()
        self.moveavg_x = []
        self.moveavg_y = []
        self.moveavg_flag = 10
        self.count = 0
        self.prev_time = 0
        self.total_time = 0
        self.x, self.y, self.z = 0, 0, 0
        self.pre_x, self.pre_y, self.pre_z = 0, 0, 0
        self.atten_const = 0.2
        
        self.prev_imu = [0,0,0]
        self.imu_prev_time = 0

        #self.f = open("/home/irl/imu.txt", "w")

        self.rotation_vectors, self.transation_vectors = None, None
        self.axis = np.float32([[-1,-1,0], [-1,1,0], [1,1,0], [1,-1,0],
                                [-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1] ])  
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/color/image_raw", 
                                            Image, self.image_callback)
        self.imu_sub = rospy.Subscriber("imudata", String, self.imu_callback)
        self.euler_pub = rospy.Publisher("eulerdata", String, queue_size=30)

        if not os.path.exists('CameraCalibration.pckl'):
            print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
            exit()
        else:
            f = open('CameraCalibration.pckl', 'rb')
            (self.cameraMatrix, self.distCoeffs, _, _) = pickle.load(f)
            f.close()
            if self.cameraMatrix is None or self.distCoeffs is None:
                print("Calibration issue. Remove CameraCalibration.pckl and recalibrate your camera with calibration_ChAruco.py.")
                exit()

    def imu_callback(self, data):
        datas = str(data)
        self.imu = map(float,datas.split('*')[1].split('\\')[0].split(','))
        now_time = time.time()
        self.new_imu = self.mt.imulpf(now_time-self.prev_time, self.prev_imu, self.imu)
        imudata = "{}\t{}\n".format(self.imu, self.new_imu)
        #self.f.write(imudata)
        self.imu_prev_time = now_time
        self.prev_imu = self.new_imu

    def image_callback(self,data):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        cv2_img = aruco.drawDetectedMarkers(cv2_img, corners, borderColor=(0, 0, 255))

        if ids is not None and len(ids) > 0:
            # Estimate the posture per each Aruco marker
            # opencv 3.2.0
            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 1, self.cameraMatrix, self.distCoeffs)
            cv2_img = aruco.drawAxis(cv2_img, self.cameraMatrix, self.distCoeffs, rvec, tvec, 1)

            Rmat, jacobian = cv2.Rodrigues(rvec)
            self.x,self.y,self.z = self.mt.getleg(Rmat, self.new_imu)

        else:

            self.x, self.y, self.z = self.pre_x*self.atten_const, self.pre_y*self.atten_const, self.pre_z*self.atten_const

        # if self.count < self.moveavg_flag:
        #         self.moveavg_x.append(self.x)
        #         self.moveavg_y.append(self.y)
        # else:
        #     self.moveavg_x[self.count%self.moveavg_flag] = self.x
        #     self.moveavg_y[self.count%self.moveavg_flag] = self.y
        # self.x, self.y = self.mt.mean(self.moveavg_x),self.mt.mean(self.moveavg_y)
        # self.count += 1

        now_time = time.time()
        ts = now_time - self.prev_time
        self.x, self.y, self.z = self.mt.lpf(ts,self.pre_x, self.x), self.mt.lpf(ts,self.pre_y, self.y), self.mt.lpf(ts,self.pre_z, self.z)
        datas = '*{:.2f},{:.2f},{:.2f}\n'.format(self.x, self.y, self.z)                                         
        self.euler_pub.publish(datas)
        print(datas)
        self.total_time += ts
        self.prev_time = now_time
        self.pre_x, self.pre_y, self.pre_z = self.x, self.y, self.z

        # Press esc or 'q' to close the image window
        cv2.imshow('image', cv2_img)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            rospy.loginfo('Exit program')
            cv2.destroyAllWindows()
            #self.f.close()
            rospy.signal_shutdown('Program terminate')  

def main(args):
    ml = markerLanding()
    rospy.init_node('markerLanding', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
