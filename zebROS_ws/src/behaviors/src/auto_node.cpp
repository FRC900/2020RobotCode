#include "ros/ros.h"
#include "behavior_actions/AutoMode.h"

//variables to store stuff read from config
int auto_mode;
std::vector<std::string> auto_steps; //stores string of action names to do, read from the auto mode array in the config file

bool auto_running = false;
//set to true when enter auto mode (not set false when enter teleop b/c we'll let this node keep running)
//set false if something fails
//set false if driver overrides auto continuing (during teleop)
//Everything checks if this is true before proceeding.


int main(int argc, char** argv)
{
	//SETUP --------------------------------------------------------------------------------------------

	//create node
	ros::init(argc, argv, "auto_node");
	ros::NodeHandle nh;

	//subscribers
	//TODO
	//rio match data (to know if we're in auto period)
	//dashboard (to get auto mode)

	//actionlib clients
	//TODO


	//other variables
	ros::Rate r(10); //TODO: what rate?


	//WAIT FOR MATCH TO START --------------------------------------------------------------------------
	ROS_INFO("Auto node - waiting for autonomous to start");

	auto_running = true; //TODO remove
	//wait for auto period to start
	while(!auto_running)
	{
		if(!ros::ok())
		{
			ROS_ERROR("Killing auto mode b/c ros not ok");
			return 1;
		}

		ros::spinOnce(); //spin so the subscribers can update
		r.sleep();
	}




	//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

	//read sequence of actions
	ROS_INFO("Auto node - reading config values based on the selected auto mode");
	nh.getParam("auto_mode_1", auto_steps); //TODO if it failed


	//run through actions in order
	for(size_t i = 0; i < auto_steps.size(); i++){
		if(auto_running)
		{
			ROS_INFO_STREAM("Auto node - running step " << i << ": " << auto_steps[i]);

			//read data needed to carry out the action
			XmlRpc::XmlRpcValue action_data;
			nh.getParam(auto_steps[i], action_data);

			//figure out what to do based on the action type
			const char* action_type = action_data["type"];
			switch(action_type) //TODO can't do switch like this
			{
				case "intake_actionlib_server":
					ROS_INFO("intake actionlib server");
					break;
				default:
					ROS_INFO_STREAM("Invalid type of action: " << action_type);
			}
		}

	}

}









//subscriber callback for match data




//subscriber callback for dashboard data
void autoModeCallback(const behavior_actions::AutoMode::ConstPtr& msg)
{
	//only change auto mode if auto hasn't started
	if(!auto_running)
	{
		//TODO read data
	}
}
