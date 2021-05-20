cmake_minimum_required(VERSION 3.0.2)
project(landing)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES landing
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS src/main.py src/calibrate.py src/generate_charuco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
