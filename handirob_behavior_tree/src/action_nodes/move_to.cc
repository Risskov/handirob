#include "handirob_behavior_tree/action_nodes/move_to.h"

MoveTo::MoveTo(const std::string& name, const BT::NodeConfiguration& config):
	CoroActionNode(name, config), nh_("~"), halt_requested_(false), ac("move_base", true)
{	
	//wait for the action server to come up
  	while(!ac.waitForServer(ros::Duration(5.0))){
    	ROS_INFO("Waiting for the move_base action server to come up");
  	}
}

BT::PortsList MoveTo::providedPorts()
{
	// This action has a single input port called "waypoint" in reference frame "map"
	// Any port must have a name. The type is optional.
    return{ BT::InputPort<geometry_msgs::Pose>("waypoint") };	// String to encode mood information.
}

BT::NodeStatus MoveTo::tick()
{
    BT::Optional<geometry_msgs::Pose> waypoint = getInput<geometry_msgs::Pose>("waypoint");
    // Check if optional is valid. If not, throw its error
    if (!waypoint)
    {
        throw BT::RuntimeError("missing required input [waypoint]: ", waypoint.error() );
		return BT::NodeStatus::FAILURE;
    }
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = waypoint.value();
	goal.target_pose.pose.position.z = 0;
	ac.sendGoal(goal);
    setStatusRunningAndYield();

  	while(ac.getState() == actionlib::SimpleClientGoalState::PENDING || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
	{
		if (!halt_requested_ || !ros::ok())
		{
			return BT::NodeStatus::FAILURE;
		}
		setStatusRunningAndYield();
	}
    return BT::NodeStatus::SUCCESS;
}

void MoveTo::halt()
{
    halt_requested_ = true;
	ac.cancelAllGoals();
}

