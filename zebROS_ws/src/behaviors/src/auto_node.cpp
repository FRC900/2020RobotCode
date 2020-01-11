#include "ros/ros.h"


//variables to store stuff read from config
int auto_mode;




int main(int argc, char** argv){
	//create node
	ros::init(argc, argv, "auto_node");
	ros::NodeHandle nh;

	//subscribers
	//rio match data (to know if we're in auto period)
	//dashboard (to get auto mode)


	//other variables
	bool auto_running = false;
		//set to true when enter auto mode (not set false when enter teleop b/c we'll let this node keep running)
		//set false if something fails
		//set false if driver overrides auto continuing (during teleop)
		//Everything checks if this is true before proceeding.
	
	ros::Rate r(10); //TODO: what rate?


	//wait for auto period to start
	while(!auto_running){
		if(!ros::ok()){
			ROS_ERROR("Killing auto mode b/c ros not ok");
			return; //TODO: what int should we return?
		}


		ros::spinOnce(); //spin so the subscriber 
		r.sleep();
	}


	//run through actions in order
	for(){
		
	
	}

}



//subscriber callback for match data




//subscriber callback for dashboard data



