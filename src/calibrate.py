#!/usr/bin/python
from logging import shutdown
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np
import glob
import os
import pickle
import sys
print(sys.version)

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 8
CHARUCOBOARD_COLCOUNT = 6 
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard_create(
	squaresX=CHARUCOBOARD_COLCOUNT,
	squaresY=CHARUCOBOARD_ROWCOUNT,
	squareLength=0.04,
	markerLength=0.02,
	dictionary=ARUCO_DICT)

class cameraCalibration:
    def __init__(self):
        # Corners discovered in all images processed
        # Aruco ids corresponding to corners discovered 
        self.corners_all = []
        self.ids_all = [] 
        self.image_size = None 
        self.capturecount = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", 
                                            Image, self.callback)
    
    def savecalibration(self):
        # Show number of valid captures
        print("{} valid captures".format(self.capturecount))

        # Make sure we were able to calibrate on at least one charucoboard
        if len(self.corners_all) == 0:
            print("Calibration was unsuccessful. We couldn't detect charucoboards in the video.")
            print("Make sure that the calibration pattern is the same as the one we are looking for (ARUCO_DICT).")
            exit()
        print("Generating calibration...")

        # Now that we've seen all of our images, perform the camera calibration
        calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=self.corners_all,
            charucoIds=self.ids_all,
            board=CHARUCO_BOARD,
            imageSize=self.image_size,
            cameraMatrix=None,
            distCoeffs=None)
                
        # Print matrix and distortion coefficient to the console
        print("Camera intrinsic parameters matrix:\n{}".format(cameraMatrix))
        print("\nCamera distortion coefficients:\n{}".format(distCoeffs))
                
        # Save the calibrationq
        f = open('./CameraCalibration.pckl', 'wb')
        pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
        f.close()
                
        # Print to console our success
        print('Calibration successful. Calibration file created: {}'.format('CameraCalibration.pckl'))

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            image = gray,
            dictionary = ARUCO_DICT
        )

        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(
                image = cv_image,
                corners = corners
            )

            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=CHARUCO_BOARD)

            # Pause to display each image, waiting for key press
            cv2.imshow('Charuco board', cv_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                rospy.loginfo('Total {} capture images for Calibration'.format(self.capturecount))
                cv2.destroyAllWindows()
                rospy.signal_shutdown('Program terminate')
            elif key == ord('s'):
                if response > 20:
                    # Add these corners and ids to our calibration arrays
                    self.corners_all.append(charuco_corners)
                    self.ids_all.append(charuco_ids)
                    
                    # Draw the Charuco board we've detected to show our calibrator the board was properly detected
                    cv_image = aruco.drawDetectedCornersCharuco(
                        image=cv_image,
                        charucoCorners=charuco_corners,
                        charucoIds=charuco_ids)
                
                    # If our image size is unknown, set it now
                    if not self.image_size:
                        self.image_size = gray.shape[::-1]
                    
                    # Reproportion the image, maxing width or height at 1000
                    proportion = max(cv_image.shape) / 1000.0
                    cv_image = cv2.resize(cv_image, (int(cv_image.shape[1]/proportion),
                                                    int(cv_image.shape[0]/proportion)))
                    
                    self.capturecount += 1
                    rospy.loginfo('Detected 20 markers and SAVE : {}'.format(self.capturecount))
                    
        else:
            cv2.imshow('Charuco board', cv_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                rospy.loginfo('Total {} capture images for Calibration'.format(self.capturecount))
                cv2.destroyAllWindows()
                rospy.signal_shutdown('Program terminate')

def main(args):
    ic = cameraCalibration()
    rospy.init_node('cameraCalibration', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    ic.savecalibration()

if __name__ == '__main__':
    main(sys.argv)
