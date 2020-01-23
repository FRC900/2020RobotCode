#include "ros/ros.h"
#include "behavior_actions/AutoMode.h"

//variables to store stuff read from config
int auto_mode = -1; //-1 if nothing selected
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
			ROS_ERROR("Killing auto node b/c ros not ok when waiting for autonomous to start");
			return 0;
		}

		ros::spinOnce(); //spin so the subscribers can update
		r.sleep();
	}




	//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

	auto_mode = 1; //TODO remove this later

	if(auto_mode < 0){
		ROS_ERROR("Auto node - No auto mode selected");
		return 1;
	}

	ROS_INFO_STREAM("Auto node - Executing auto mode " << auto_mode);

	//read sequence of actions
	if(! nh.getParam("auto_mode_" + std::to_string(auto_mode), auto_steps)){
		ROS_ERROR_STREAM("Couldn't read auto_mode_" + std::to_string(auto_mode) + " config value in auto node");
		return 1; //TODO pick a default?
	}


	//run through actions in order
	for(size_t i = 0; i < auto_steps.size(); i++){
		if(auto_running)
		{
			ROS_INFO_STREAM("Auto node - running step " << i << ": " << auto_steps[i]);

			//read data needed to carry out the action
			XmlRpc::XmlRpcValue action_data;
			if(! nh.getParam(auto_steps[i], action_data)){
				ROS_ERROR_STREAM("Auto node - Couldn't read data for '" << auto_steps[i] << "' auto action from config file");
				return 1;
			}

			//figure out what to do based on the action type, and do it
			if(action_data["type"] == "intake_actionlib_server")
			{
				//do stuff
			}
			else if(action_data["type"] == "pause")
			{
				//do stuff
			}
			else
			{
				ROS_ERROR_STREAM("Invalid type of action: " << action_data["type"]);
				return 1;
			}
		}

	}

	ROS_INFO("Autonomous actions completed!");

	return 0;
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
