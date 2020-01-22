#include "ros/ros.h"
#include "behavior_actions/AutoMode.h" //TODO dependencies for this


//variables to store stuff read from config
int auto_mode;
std::vector</*string*/> auto_step_names;
std::vector</*something*/> auto_step;



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
	int auto_mode = -1; //-1 indicates that no auto mode has been picked - don't execute anything //TODO reevaluate this

	bool auto_running = false;
	//set to true when enter auto mode (not set false when enter teleop b/c we'll let this node keep running)
	//set false if something fails
	//set false if driver overrides auto continuing (during teleop)
	//Everything checks if this is true before proceeding.

	ros::Rate r(10); //TODO: what rate?


	//WAIT FOR MATCH TO START --------------------------------------------------------------------------

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

	//run through actions in order
	for(){
		if(auto_running)
		{

		}

	}

}



//subscriber callback for match data




//subscriber callback for dashboard data
void autoModeCallback(const behavior_actions::AutoMode::ConstPtr& msg)
{
	//read msg data
	//TODO do it

	//only change auto mode and reread config
	if(/*new auto mode*/ != auto_mode){
		auto_mode = /*new auto mode*/
	}
	else {
		return;
	}


	//if no auto mode selected, don't do anything
	if(auto_mode == -1)
	{
		return;
	}

	//read details of the auto sequence
	//TODO probably with the XmlRpc method??
}
