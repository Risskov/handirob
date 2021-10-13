# HanDiRob

## On startup
On startup a simple roscore will start including two additional nodes: 
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR): This node starts the openCR board which makes it possible to start teleoperation of the robot, read its sensors and see the state of the battery.
- [Robot_state_publisher](http://wiki.ros.org/robot_state_publisher): This publishes the transforms of each component of the robot and how they relate to eachother. Can be visualized in Rviz.  
**WARN!** The roscore that starts on startup is not always functioning correctly if you try to access it from a remote pc. If anything unusual happens or to be sure run the following commands:  
`rosrestart`
This will kill the currently running roscore and start a new with the beforementioned nodes.

## Start sensors
To start the lidars and their merging, as well as the camera run the following:  
`roslaunch handirob_bringup handirob_sensors.launch`  
Individual nodes can be excluded using `dual_lidar:=false`, `camera:=false` or `pointcloud:=false`  
The pointcloud is disabled by default

### Teleoperate the robot
To control the robot with a keyboard run:  
`roslaunch handirob_bringup handirob_teleop_key.launch`  
You can then control the robot with `wasd` and stop it completely with `space`.

### Launch navigation stack with a saved map

`roslaunch handirob_navigation handirob_navigation.launch`  
The default loaded map is currently SDU_lab_top.yaml.  
If another map is to be used add `map_server:=$(link to map.yaml file)` to the command.  

## Rviz
To see the map and set initial positionts and goals to the robot, start Rviz on a remote pc with ROS installed as well as the correct ROS_MASER_URI and ROS_HOSTNAME ip exportet, and run:  
`rosrun rviz rviz -d /../$(location_of_rviz_file)/handirob_navigation.rviz`  
A GUI will open.  
Give the robot an initial 2D pose estimate, by placing the green arrow from the top, the approximate position and orientation of the robot in the map. Teleoperate the robot for a bit until the lidar points line up with the map, and the localization arrows are as close together as possible.  
It is now possible to give it a 2D Nav goal, which the robot will try to reach to its best abilities.  

### Launch Cartographer SLAM
If a new map is needed to be generated run the following SLAM command  
`roslaunch handirob_bringup handirob_cart_nav.launch`  
Additionally start teleoperation to move the robot around and it will start mapping.  
When done mapping, save the map with:  
`rosrun map_server map_saver -f $(file_name)`  

## Docking and lifting
To start the Lifting server alone run:  
`roslaunch handirob_lifting lifting_server.launch`  
The lift will then be lowered and can be lifted or lowered manually by:  
`rosservice call /lifting_server/lift_module`  
`rosservice call /lifting_server/lower_module`  
If docking is needed as well run:  
`roslaunch handirob_docking handirob_docking.launch`  
This will start both the docking and lifting server that can look for a stand.  
To start the docking process call:  
`rosrun handirob_docking docking_client.py`  
The robot will then start looking for a stand in front of it, and will dock with it if it is witihin a short distance.  












