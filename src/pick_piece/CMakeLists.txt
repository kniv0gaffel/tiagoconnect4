cmake_minimum_required(VERSION 3.0.2)
project(pick_piece)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_ros_planning_interface
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs moveit_ros_planning_interface
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)


add_executable(pick_piece src/pick_piece.cpp)

target_link_libraries(pick_piece ${catkin_LIBRARIES})
