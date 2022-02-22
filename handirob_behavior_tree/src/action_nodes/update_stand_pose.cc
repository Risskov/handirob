#include "handirob_behavior_tree/action_nodes/update_stand_pose.h"


// rosservice call /ui_control/setmood "angry"

UpdateStandPose::UpdateStandPose(const std::string& name, const BT::NodeConfiguration& config):
	SyncActionNode(name, config), nh_("~")
{	
	ros::Publisher stand_pub = nh_.advertise<stand_pose::Stand>("/stand_detector/Stand", 10);
}


BT::PortsList UpdateStandPose::providedPorts()
{
	// This action has a single input port called "mood"
	// Any port must have a name. The type is optional.
    return{ BT::InputPort<std::string>("standID"), 
	BT::InputPort<geometry_msgs::Pose>("pose"), 
	BT::InputPort<bool>("pose_know")};	// String to encode mood information.
}

BT::NodeStatus UpdateStandPose::tick()
{
    BT::Optional<std::string> standID = getInput<std::string>("standID");
	BT::Optional<geometry_msgs::Pose> pose = getInput<geometry_msgs::Pose>("standID");
	BT::Optional<bool> pose_known = getInput<bool>("known");
    // Check if optional is valid. If not, throw its error
    if (!standID)
        throw BT::RuntimeError("missing required input [standID]: ", standID.error() );
	if (!pose)
        throw BT::RuntimeError("missing required input [Pose]: ", pose.error() );
	if (!pose_known)
        throw BT::RuntimeError("missing required input [poseKnown]: ", pose_known.error() );

	stand_pose::Stand msg = stand_pose::Stand();
	msg.pose = (pose_known.value() ? pose.value(): geometry_msgs::Pose());
	msg.header.stamp = ros::Time::now();
	msg.id = standID.value();
	msg.known = pose_known.value();
	stand_pub.publish(msg);

    return BT::NodeStatus::SUCCESS;
}
