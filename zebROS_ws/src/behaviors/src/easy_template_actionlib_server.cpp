//THIS IS A TEMPLATE FILE. TO USE, COPY IT AND REPLACE:
// ServerName	with your server's name, e.g. CargoIntake
// server_name	with your server's name, e.g. cargo_intake
// Thing		with the name of your action file (if file is Intake.action, replace w/ Intake)


#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behaviors/ThingAction.h"

//include controller service files and other service files
// e.g. #include "controller_package/ControllerSrv.h"
// e.g. #include "sensor_msgs/JointState.h" -has linebreak sensor data FYI

//config variables, with defaults
double server_timeout = ; //overall timeout for your server
double wait_for_server_timeout = ; //timeout for waiting for other actionlib servers to become available before exiting this one

//create the class for the actionlib server
class ServerNameAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::ThingAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call other actionlib servers
//		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		//clients to call controllers
		//e.g. ros::ServiceClient mech_controller_client_; //create a ros client to send requests to the controller

		//variables to store if server was preempted or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted;
		bool timed_out;
		ros::Rate r(100); //used for wait loops
		double start_time;

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		ServerNameAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ServerNameAction::executeCB, this, _1), false),
			action_name_(name)
			//ac_elevator_("/elevator/elevator_server", true) example how to initialize other action clients, don't forget to add a comma on the previous line
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		//e.g. mech_controller_client_ = nh_.serviceClient<controller_package::ControllerSrv>("name_of_service", false, service_connection_header);

	}

		~ServerNameAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::ThingGoalConstPtr &goal)
		{
			ROS_WARN("%s: Running callback", action_name_.c_str());

			start_time = ros::Time::now().toSec();
			preempted = false;
			timed_out = false;



			//wait for all actionlib servers we need
			/* e.g.
			if(!ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("server_name_server couldn't find elevator actionlib server");
				as_.setPreempted();
				return;
			}
			*/

			//wait for all controller servers we need
			/* e.g.
			if(! mech_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("server_name_server can't find mech_controller");
				as_.setPreempted();
				return;
			}
			*/





			//Basic controller call ---------------------------------------
			if(!preempted && !timed_out && ros::ok())
			{
				ROS_INFO("server_name_server: what this is doing");
				//call controller client, if failed set preempted = true, and log an error msg


				//if necessary, run a loop to wait for the controller to finish
				while(!preempted && !timed_out && ros::ok())
				{
					//check preempted
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR("server_name_server: preempt in ______");
						preempted = true;
					}
					//test if succeeded, if so, break out of the loop
					else if(test here) {
						break;
					}
					//check timed out
					else if (ros::Time::now().toSec() - start_time > server_timeout) {
						ROS_ERROR("server_name_server: timed out in ______");
						timed_out = true;
					}
					//otherwise, pause then loop again
					else {
						r.sleep();
					}
				}
			}
			//preempt handling (skip this and set final state at end if only 1 possble final state)
			/*
			if(preempted || timed_out || !ros::ok())
			{}
			*/

			//if necessary, pause a bit between doing things (between piston firings usually)
			//ros::Duration(sec_to_pause).sleep();







			//Basic actionlib server call -------------------------------------
			if(!preempted && !timed_out && ros::ok())
			{
				ROS_INFO("what this is doing");
				//Call actionlib server
				/* e.g.
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);
				*/
				//wait for actionlib server
				//e.g. waitForActionlibServer(ac_elevator_, 30, "calling elevator server"); //this method defined below. Args: action client, timeout in sec, description of activity
			}
			//preempt handling or pause if necessary (see basic controller call)





			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that


			//log result and set actionlib server state appropriately
			behaviors::ThingResult result;

			if(preempted) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_WARN("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);
			}

			return;

		}

		void waitForController(std::string activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called
		{
			while(!preempted && !timed_out && ros::ok())
			{
				//check preempted
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempt - " << activity);
					preempted = true;
				}
				//test if succeeded, if so, break out of the loop
				else if(test here) {
					break;
				}
				//check timed out
				else if (ros::Time::now().toSec() - start_time > server_timeout) {
					ROS_ERROR(action_name_ << ": timed out - " << activity);
					timed_out = true;
				}
				//otherwise, pause then loop again
				else {
					r.sleep();
				}
			}
		}

		void waitForActionlibServer(auto &action_client, double timeout, std::string activity)
		{
			double request_time = ros::Time::now().toSec();

			//wait for actionlib server to finish
			std::string state;
			while(!preempted && !timed_out && ros::ok())
			{
				state = action_client.getState().toString();

				if(state == "PREEMPTED") {
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server returned preempted during " << activity);
					preempted = true;
				}
				//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
				else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
						(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
				{
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server timed out during " << activity);
					timed_out = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
					break; //stop waiting
				}
				//checks related to this file's actionlib server
				else if (as_.isPreemptRequested()) {
					ROS_ERROR_STREAM(action_name_ << ": preempted during " << activity);
					preempted = true;
				}
				else if (ros::Time::now().toSec() - start_time > server_timeout) {
					ROS_ERROR_STREAM(action_name_ << ": timed out during " << activity);
					timed_out = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else { //if didn't succeed and nothing went wrong, keep waiting
					r.sleep();
				}
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "server_name_server");

	//create the actionlib server
	ServerNameAction server_name_action("server_name_server");

	//get config values
	ros::NodeHandle n;
	//ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params"); //node handle for a lower-down namespace

	/* e.g.
	if (!n.getParam("/teleop/teleop_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");
	*/


	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
