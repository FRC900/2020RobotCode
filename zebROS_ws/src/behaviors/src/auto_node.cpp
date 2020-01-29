#include "ros/ros.h"
#include "std_msgs/String.h"
#include "behavior_actions/AutoMode.h" //msg file
#include "behavior_actions/StopAuto.h" //srv file
#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"
#include "behavior_actions/ElevatorAction.h" //TODO remove this, it's for testing using last year's stuff

#include "thread"
#include "atomic"

//VARIABLES ---------------------------------------------------------
int auto_mode = -1; //-1 if nothing selected
std::vector<std::string> auto_steps; //stores string of action names to do, read from the auto mode array in the config file

bool auto_started = false; //set to true when enter auto time period (not set false when enter teleop b/c we'll let this node keep running)
bool auto_stopped = false; //set to true if driver stops auto (see the service, calls stopAuto() )
//All actions check if(auto_started && !auto_stopped) before proceeding.

std::atomic<int> auto_state(0); //This state is published by the publish thread
//0:not ready, 1:ready, 2:running, 3:done, -1:error


//FUNCTIONS -------
//function to wait while an actionlib server is running
//TODO: steal from actionlib server template maybe




//server callback for stop autonomous execution
bool stopAuto(behavior_actions::StopAuto::Request &req,
			  behavior_actions::StopAuto::Response &res)
{
	ROS_INFO("Stopping auto execution");
	auto_stopped = true;
	return true;
}

//subscriber callback for match data
void matchDataCallback(const frc_msgs::MatchSpecificData::ConstPtr& msg)
{
	if(msg->Autonomous)
	{
		auto_started = true; //only want to set this to true, never set it to false afterwards
	}
}



//subscriber callback for dashboard data
void updateAutoMode(const behavior_actions::AutoMode::ConstPtr& msg)
{
	auto_mode = msg->auto_mode;
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
	//rio match data (to know if we're in auto period)
	ros::Subscriber match_data_sub = nh.subscribe("/frcrobot_rio/match_data", 1, matchDataCallback);
	//dashboard (to get auto mode)
	ros::Subscriber auto_mode_sub = nh.subscribe("auto_mode", 1, updateAutoMode); //TODO get correct topic name

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

	//wait for auto period to start
	while( (!auto_started && !auto_stopped) || auto_mode <= 0) //last one checks if we selected an auto mode yet
	{
		if(!ros::ok()){
			ROS_ERROR("Killing auto node b/c ros not ok when waiting for autonomous to start");
			auto_state = 3; //done
			auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
			return 0;
		}

		ros::spinOnce(); //spin so the subscribers can update

		if(auto_mode > 0){
			auto_state = 1; //ready
		}
		if(auto_started && auto_mode <= 0){
			ROS_ERROR("Please choose an auto mode");
		}

		r.sleep();
	}

	if(auto_stopped){
		ROS_INFO("Auto stopped before execution");
		auto_state = 3; //done
		auto_state_pub_thread.join();
		return 0;
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
		if(auto_started && !auto_stopped)
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

	if(auto_stopped){
		ROS_INFO("Autonomous actions stopped before completion");
	}
	else {
		ROS_INFO("Autonomous actions completed!");
	}
	auto_state = 3; //done
	auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
	return 0;
}

