cmake_minimum_required(VERSION 2.8.3)
project(handirob_docking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

add_action_files(
DIRECTORY action
FILES Docking.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)