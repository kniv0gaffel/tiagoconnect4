cmake_minimum_required(VERSION 3.0.2)
project(pupil_size_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(pupil_size_publisher src/pupil_size_publisher.cpp)

target_link_libraries(pupil_size_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
