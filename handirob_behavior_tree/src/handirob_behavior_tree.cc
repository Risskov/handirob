#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <ros/ros.h>

#include "handirob_behavior_tree/action_nodes/set_mood.h"

/*
#include "smooth_uc3_behaviour_tree/action_nodes/setMood.h"
#include "smooth_uc3_behaviour_tree/action_nodes/turn_executer.h"
#include "smooth_uc3_behaviour_tree/action_nodes/detect_people.h"
*/

int main(int argc, char **argv)
{
	BT::BehaviorTreeFactory factory;

	ros::init(argc, argv, "handirob_behavior_tree");
	ros::NodeHandle nh("~");

	factory.registerNodeType<SetMood>("SetMood");
	/*
	factory.registerNodeType<Turn>("Turn");
	factory.registerNodeType<TurnExecuter>("TurnExecuter");
	factory.registerNodeType<DetectPeople>("DetectPeople");
	*/


	std::string behavior_tree_file_path;
	nh.param<std::string>("behavior_tree_file_path", behavior_tree_file_path, 
		"src/handirob_behavior_tree/trees/testtree.xml");

	std::cout << behavior_tree_file_path << std::endl;

	auto tree = factory.createTreeFromFile(behavior_tree_file_path);


	BT::NodeStatus status = BT::NodeStatus::RUNNING;

	double start_time = ros::Time::now().toNSec();




    while (status == BT::NodeStatus::RUNNING ) // || status == BT::NodeStatus::FAILURE
    {
        ros::spinOnce();
        status = tree.tickRoot();
        // std::cout << "Status: " << status << "|Time: " << (ros::Time::now().toNSec() - start_time)/1000.0 << std::endl;
        start_time = ros::Time::now().toSec();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
