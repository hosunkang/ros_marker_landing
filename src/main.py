#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

bridge = CvBridge()

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)
rotation_vectors, transation_vectors = None, None
axis = np.float32([[-1,-1,0], [-1,1,0], [1,1,0], [1,-1,0],
                    [-1,-1,1],[-1,1,1],[1,1,1],[1,-1,1] ])  


def image_callback(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        cv2.circle(cv2_img, (50,50), 10, 255)
        cv2.imshow('image', cv2_img)
    except CvBridgeError, e:
        print(e)

def main():
    cv2.namedWindow('image')
    cv2.startWindowThread()

    rospy.init_node('image_listener')
    image_topic = "camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
