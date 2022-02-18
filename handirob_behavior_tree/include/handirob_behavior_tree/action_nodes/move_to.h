#ifndef MOVE_TO_H
#define MOVE_TO_H

// ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

// others
#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveTo : public BT::CoroActionNode
{  
public:
    MoveTo(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override; 
	void halt() override;

private:
    ros::NodeHandle nh_;
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac;
	bool halt_requested_;
};


#endif // MOVE_TO_H