#!/usr/bin/python
import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
import os, pickle
import serial
import serial.tools.list_ports_linux as sp
import math

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

def ra2de(ra):
    return ra*180/math.pi

def de2ra(de):
    return de*math.pi/180

def Rmat2euler_yxz(rmat):
    [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
    if r23 < 1:
        if r23 > -1:
            x = math.asin(-r23)
            y = math.atan2(r13,r33)
            z = math.atan2(r21,r22)
        else:
            x = math.pi/2
            y = -math.atan2(-r12,r11)
            z = 0
    else:
        x = -math.pi/2
        y = math.atan2(-r12,r11)
        z = 0
    return x,y,z

def Rmat2euler_zyx(rmat):
    [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
    if r31 < 1:
        if r31 > -1:
            x = math.atan2(r32,r33)
            y = math.asin(-r31)
            z = math.atan2(r21,r11)
        else:
            x = 0
            y = math.pi/2
            z = -math.atan2(-r23,r22)
    else:
        x = 0
        y = -math.pi/2
        z = math.atan2(-r23,r22)
    return ra2de(x), ra2de(y), ra2de(z)    

def rotationMatrix(angle, flag):
    if flag == 'x':
        mat = np.array([[1,0,0],
                        [0,round(math.cos(angle),3),-round(math.sin(angle),3)],
                        [0,round(math.sin(angle),3),round(math.cos(angle),3)]])
    elif flag == 'y':
        mat = np.array([[round(math.cos(angle),3),0,round(math.sin(angle),3)],
                        [0,1,0],
                        [-round(math.sin(angle),3),0,round(math.cos(angle),3)]])
    elif flag == 'z':
        mat = np.array([[round(math.cos(angle),3),-round(math.sin(angle),3),0],
                        [round(math.sin(angle),3),round(math.cos(angle),3),0],
                        [0,0,1]])
    return mat

Rxpi = rotationMatrix(math.pi, 'x')
Rypi = rotationMatrix(math.pi, 'y')

def getleg(Rcm,imu):
    imu = [de2ra(imu[0]), de2ra(-imu[1]), de2ra(-imu[2])]
    Rlb = np.dot(np.dot(Rxpi,rotationMatrix(imu[1], 'y')), rotationMatrix(imu[0],'x'))
    Rbc = Rypi
    Rlm = np.dot(np.dot(Rlb,  Rbc), Rcm)
    a,b,c = Rmat2euler_yxz(Rlm)
    #Rlt1 = np.dot(np.dot(rotationMatrix(b, 'y'), rotationMatrix(a, 'x')), Rxpi)
    Rlt1 = np.dot(np.dot(Rlm, rotationMatrix(-c, 'z')), Rxpi)
    Rlt2 = np.dot(Rlt1, rotationMatrix(math.pi,'z'))

    return Rmat2euler_zyx(Rlt1)

def mean(datas):
    return sum(datas)/len(datas)

class markerLanding:
    def __init__(self):
        self.moveavg_x = []
        self.moveavg_y = []
        self.moveavg_flag = 10
        self.count = 0
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
            
            x,y,z = getleg(Rmat, self.imu)

            if self.count < self.moveavg_flag:
                self.moveavg_x.append(x)
                self.moveavg_y.append(y)
            else:
                self.moveavg_x[self.count%self.moveavg_flag] = x
                self.moveavg_y[self.count%self.moveavg_flag] = y
            self.count += 1
            datas = '*{:.2f},{:.2f},{:.2f}\n'.format(mean(self.moveavg_x),mean(self.moveavg_y),z)
            self.euler_pub.publish(datas)
            print(datas)

        # Press esc or 'q' to close the image window
        cv2.imshow('image', cv2_img)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            rospy.loginfo('Exit program')
            cv2.destroyAllWindows()
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
