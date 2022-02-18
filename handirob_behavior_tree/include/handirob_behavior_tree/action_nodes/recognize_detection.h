#ifndef RECOGNIZE_DETECTION_H
#define RECOGNIZE_DETECTION_H

// ROS
#include <ros/ros.h>
#include <stand_pose/Stand.h>

// Other
#include <string.h>

// Behavior tree
#include "behaviortree_cpp_v3/behavior_tree.h"

class RecognizeDetection : public BT::CoroActionNode
{  
public:
	RecognizeDetection(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override; 
	void halt() override;
	

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub;
	void standCallback_(stand_pose::Stand stand_);
	bool detected_ = false;
	std::string stand_id_;
	bool halt_requested_;
};



#endif // RECOGNIZE_DETECTION_H