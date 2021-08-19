# HanDiRob

## Run on robot

### Launch opencr, lidar, camera and pointcloud nodes
`roslaunch handirob_bringup handirob_robot.launch`
Individual nodes can be excluded using `lidar:=false`, `camera:=false` or `pointcloud:=false`

## Run on remote

### Teleoperate the robot

`roslaunch handirob_bringup handirob_teleop_key.launch`

### Launch Cartographer with a saved map

`roslaunch handirob_navigation handirob_navigation.launch`

### Launch Cartographer and navigation

`roslaunch handirob_bringup handirob_cart_nav.launch`
