cmake_minimum_required(VERSION 2.8.3)
project(handirob_detection)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  object_msgs
  sensor_msgs
)

catkin_python_setup()


catkin_package()
