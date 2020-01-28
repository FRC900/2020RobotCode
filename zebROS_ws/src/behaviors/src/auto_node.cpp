#include "ros/ros.h"
#include "std_msgs/String.h"
#include "behavior_actions/AutoMode.h" //msg file
#include "behavior_actions/StopAuto.h" //srv file

#include "actionlib/client/simple_action_client.h"
#include "behavior_actions/ElevatorAction.h" //TODO remove this, it's for testing using last year's stuff

#include "thread"
#include "atomic"

//VARIABLES --------
int auto_mode = -1; //-1 if nothing selected
std::vector<std::string> auto_steps; //stores string of action names to do, read from the auto mode array in the config file

bool auto_running = false;
//set to true when enter auto mode (not set false when enter teleop b/c we'll let this node keep running)
//set false if something fails
//set false if driver overrides auto continuing (during teleop)
//Everything checks if this is true before proceeding.

std::atomic<int> auto_state(0); //This state is published by the publish thread
//0:not ready, 1:ready, 2:running, 3:done, -1:error


//FUNCTIONS -------
//function to wait while an actionlib server is running
//TODO: steal from actionlib server template maybe




//server callback for stop autonomous execution
bool stopAuto(behavior_actions::StopAuto::Request &req,
			  behavior_actions::StopAuto::Response &res)
{
	auto_running = false;
	return true;
}

//subscriber callback for match data




//subscriber callback for dashboard data
void updateAutoMode(const behavior_actions::AutoMode::ConstPtr& msg)
{
	//only change auto mode if auto hasn't started - don't want to try to change it in the middle
	if(!auto_running)
	{
		auto_mode = msg->auto_mode;
	}
}




//function to publish auto node state (run on a separate thread)
//this is read by the dashboard to display it to the driver
void publishAutoState(ros::Publisher publisher)
{
	ros::Rate r(10); //TODO config
	std_msgs::String msg;

	while(ros::ok()){
		switch(auto_state){
			case -1: msg.data = "Error"; break;
			case 0: msg.data = "Not Ready"; break;
			case 1: msg.data = "Ready"; break;
			case 2: msg.data = "Running"; break;
			case 3: msg.data = "Done"; break;
		}
		publisher.publish(msg);
		r.sleep();
	}
}


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

	//publishers
	//auto state
	ros::Publisher state_pub = nh.advertise<std_msgs::String>("auto_state", 1);
	std::thread auto_state_pub_thread(publishAutoState, state_pub);

	//servers
	ros::ServiceServer stop_auto_server = nh.advertiseService("stop_auto", stopAuto); //called by teleoop node to stop auto execution during teleop if driver wants

	//actionlib clients
	//TODO
	//example:
	actionlib::SimpleActionClient<behavior_actions::ElevatorAction> elevator_ac("/elevator/elevator_server", true);


	//other variables
	ros::Rate r(10); //TODO: what rate?


	//WAIT FOR MATCH TO START --------------------------------------------------------------------------
	ROS_INFO("Auto node - waiting for autonomous to start");

	ros::Duration(5).sleep(); //TODO remove
	auto_mode = 1; //TODO remove
	auto_running = true; //TODO remove

	//wait for auto period to start
	while(!auto_running)
	{
		if(!ros::ok())
		{
			ROS_ERROR("Killing auto node b/c ros not ok when waiting for autonomous to start");
			auto_state = 3; //done
			auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
			return 0;
		}

		if(auto_mode > 0){
			auto_state = 1; //ready
		}

		ros::spinOnce(); //spin so the subscribers can update
		r.sleep();
	}

	//check if an auto mode was selected
	if(auto_mode < 0){
		ROS_ERROR("Auto node - No auto mode selected");
		auto_state = -1; //error
		auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
		return 1;
	}


	//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

	ROS_INFO_STREAM("Auto node - Executing auto mode " << auto_mode);
	auto_state = 2; //running

	//read sequence of actions from config
	if(! nh.getParam("auto_mode_" + std::to_string(auto_mode), auto_steps)){
		ROS_ERROR_STREAM("Couldn't read auto_mode_" + std::to_string(auto_mode) + " config value in auto node");
		auto_state = -1;
		auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
		return 1;
	}


	//run through actions in order
	for(size_t i = 0; i < auto_steps.size(); i++){
		if(auto_running)
		{
			ROS_INFO_STREAM("Auto node - running step " << i << ": " << auto_steps[i]);

			//read data from config needed to carry out the action
			XmlRpc::XmlRpcValue action_data;
			if(! nh.getParam(auto_steps[i], action_data)){
				ROS_ERROR_STREAM("Auto node - Couldn't read data for '" << auto_steps[i] << "' auto action from config file");
				auto_state = -1;
				auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
				return 1;
			}

			//figure out what to do based on the action type, and do it
			if(action_data["type"] == "intake_actionlib_server")
			{
				//do stuff
				ros::Duration(3).sleep();
			}
			else if(action_data["type"] == "pause")
			{
				//do stuff
				ros::Duration(3).sleep();
			}
			//TODO remove test
			else if(action_data["type"] == "elevator_actionlib_server")
			{
				if(!elevator_ac.waitForServer(ros::Duration(5))){ROS_ERROR("couldn't find server");} //for some reason this is necessary, even if the server has been up and running for a while
				behavior_actions::ElevatorGoal goal;
				goal.setpoint_index = (int) action_data["goal"]["setpoint_index"];
				elevator_ac.sendGoal(goal);
			}
			else
			{
				ROS_ERROR_STREAM("Invalid type of action: " << action_data["type"]);
				auto_state = -1;
				auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
				return 1;
			}
		}

	}

	ROS_INFO("Autonomous actions completed!");
	auto_state = 3; //done
	auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
	return 0;
}

