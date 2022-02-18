#ifndef SET_MOOD_H
#define SET_MOOD_H

// ROS
#include <ros/ros.h>

// others
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string.h>
#include "ui_control/ui_mood.h"


class SetMood : public BT::SyncActionNode
{  
public:
    SetMood(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override; 

private:
    ros::NodeHandle nh_;
	ros::ServiceClient client;
};



#endif // SET_MOOD_H