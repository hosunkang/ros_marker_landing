#!/usr/bin/python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
import os, pickle

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

class markerLanding:
    def __init__(self):
        self.rotation_vectors, self.transation_vectors = None, None
        self.axis = np.float32([[-1,-1,0], [-1,1,0], [1,1,0], [1,-1,0],
                                [-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1] ])  
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/color/image_raw", 
                                            Image, self.image_callback)
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
    
    def drawCube(self, corners, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)

        # draw ground floor in green
        # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

        # draw pillars in blue color
        for i,j in zip(range(4),range(4,8)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

        # draw top layer in red color
        img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
        return img


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
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                if len(sys.argv) == 2 and sys.argv[1] == 'cube':
                    try:
                        imgpts, jac = cv2.projectPoints(self.axis, rvec, tvec, self.cameraMatrix, self.distCoeffs)
                        cv2_img = self.drawCube(cv2_img, corners, imgpts)
                    except:
                        continue
                else:
                    cv2_img = aruco.drawAxis(cv2_img, self.cameraMatrix, self.distCoeffs, rvec, tvec, 1)

        # Press esc or 'q' to close the image window
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            rospy.loginfo('Exit program')
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Program terminate')

        cv2.imshow('image', cv2_img)

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
