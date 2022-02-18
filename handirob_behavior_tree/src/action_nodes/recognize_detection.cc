#include "handirob_behavior_tree/action_nodes/recognize_detection.h"


RecognizeDetection::RecognizeDetection(const std::string& name, const BT::NodeConfiguration& config):
	CoroActionNode(name, config), nh_("~")//, sub("chatter", 1000, callback)
{	
	//Init code
	sub = nh_.subscribe<stand_pose::Stand>("/stand_detector/Stand", 10, &RecognizeDetection::standCallback_, this);
	BT::Optional<std::string> stand_id = getInput<std::string>("standID");
	if (!stand_id){
		throw BT::RuntimeError("missing required input [standID]: ", stand_id.error() );
	}
	stand_id_ = stand_id.value();
}

BT::NodeStatus RecognizeDetection::tick(){
	BT::Optional<std::string> stand_id = getInput<std::string>("standID");
	if (!stand_id){
		throw BT::RuntimeError("missing required input [standID]. got ", stand_id.error());
		return BT::NodeStatus::FAILURE;
	}
	stand_id_ = stand_id.value();
	while(!detected_){
		if (!halt_requested_ || !ros::ok())
		{
			return BT::NodeStatus::FAILURE;
		}
		setStatusRunningAndYield();
	}
	return BT::NodeStatus::SUCCESS;
}

BT::PortsList RecognizeDetection::providedPorts()
{
	// This action has a single input port called "waypoint" in reference frame "map"
	// Any port must have a name. The type is optional.
    return{ BT::InputPort<std::string>("standID") };	// String to encode mood information.
}


void RecognizeDetection::halt()
{
    halt_requested_ = true;
}

void RecognizeDetection::standCallback_(stand_pose::Stand stand_)
{
	detected_ = (stand_.id == stand_id_);
}
