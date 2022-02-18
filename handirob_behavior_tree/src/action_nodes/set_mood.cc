#include "handirob_behavior_tree/action_nodes/set_mood.h"

// rosservice call /ui_control/setmood "angry"

SetMood::SetMood(const std::string& name, const BT::NodeConfiguration& config):
	SyncActionNode(name, config), nh_("~")
{	
	client = nh_.serviceClient<ui_control::ui_mood>("/ui_control/setmood");
}


BT::PortsList SetMood::providedPorts()
{
	// This action has a single input port called "mood"
	// Any port must have a name. The type is optional.
    return{ BT::InputPort<std::string>("mood") };	// String to encode mood information.
}

BT::NodeStatus SetMood::tick()
{
    BT::Optional<std::string> msg = getInput<std::string>("mood");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", msg.error() );
    }
	ui_control::ui_mood srv;
	srv.request.mood = msg.value();
    if (client.call(srv))
		{
			ROS_INFO("service called successfully");
		}
    return BT::NodeStatus::SUCCESS;
}


