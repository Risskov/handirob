cmake_minimum_required(VERSION 3.0.2)
project(handirob_behavior_tree)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behaviortree_cpp_v3
  ui_control
  move_base_msgs
  actionlib
  geometry_msgs 
  stand_pose
)

## Behavior tree cpp v3 dependancy
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if ($ENV{ROS_DISTRO} STREQUAL "melodic")
  find_package(BehaviorTreeV3)
  find_package(behaviortree_cpp_v3) 
else ()
  find_package(BehaviorTreeV3)
endif ()


catkin_package(
  CATKIN_DEPENDS 
  behaviortree-cpp-v3 
  roscpp 
  std_msgs 
  ui_control
  move_base_msgs 
  actionlib
  geometry_msgs 
  stand_pose
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
set(INCLUDE_DIR /src)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)

add_library(set_mood ${PROJECT_SOURCE_DIR}/src/action_nodes/set_mood.cc )
target_link_libraries(set_mood BT::behaviortree_cpp_v3)
target_link_libraries(set_mood ${LIBS} ${catkin_LIBRARIES})

add_library(move_to ${PROJECT_SOURCE_DIR}/src/action_nodes/move_to.cc )
target_link_libraries(move_to BT::behaviortree_cpp_v3)
target_link_libraries(move_to ${LIBS} ${catkin_LIBRARIES})

add_library(recognize_detection ${PROJECT_SOURCE_DIR}/src/action_nodes/recognize_detection.cc )
add_dependencies(recognize_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognize_detection BT::behaviortree_cpp_v3)
target_link_libraries(recognize_detection ${LIBS} ${catkin_LIBRARIES})



add_executable(handirob_behavior_tree ${PROJECT_SOURCE_DIR}/src/handirob_behavior_tree.cc ${HEADER_FILES})
target_link_libraries(handirob_behavior_tree set_mood move_to recognize_detection BT::behaviortree_cpp_v3 ${catkin_LIBRARIES})

