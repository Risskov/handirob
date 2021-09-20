cmake_minimum_required(VERSION 2.8.3)
project(handirob_lifting)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_srvs
)

add_action_files(
  DIRECTORY action
  FILES Lifting.action Lowering.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package()
