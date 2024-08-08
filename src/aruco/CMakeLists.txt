cmake_minimum_required(VERSION 3.0.2)
project(aruco)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(PROGRAMS scripts/aruco.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
