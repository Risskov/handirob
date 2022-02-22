#ifndef UPDATE_STAND_POSE_H
#define UPDATE_STAND_POSE_H

// ROS
#include <ros/ros.h>
#include <stand_pose/Stand.h>
#include "geometry_msgs/Pose.h"

// Behavior tree
#include "behaviortree_cpp_v3/behavior_tree.h"

class UpdateStandPose : public BT::SyncActionNode
{  
public:
    UpdateStandPose(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override; 

private:
    ros::NodeHandle nh_;
	ros::ServiceClient client;
	ros::Publisher stand_pub;
};

#endif // UPDATE_STAND_POSE_H