#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/IndexerAction.h"

//include controller service files and other service files
#include "controllers_2020_msgs/IndexerSrv.h"
#include "sensor_msgs/JointState.h" //for linebreak sensor data

class Linebreak {
	public:
		std::string name_;
		size_t idx_; //index of this linebreak in the joint_states message
		int true_count_;
		int false_count_;
		int debounce_iterations_;
		bool triggered_;

		//called every time the joint state subscriber is run
		void update(const sensor_msgs::JointState &joint_state)
		{
			//set idx if it hasn't already been set
			if ( idx_ >= joint_state.name.size() ) //idx_ is infinitely big before it's set
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == name_){
						idx_ = i;
					}
				}
			}
			else {
				ROS_ERROR_STREAM("Indexer server - Linebreak named " << name_ << " not found in joint_states");
			}

			//update linebreak state
			if (joint_state.position[idx_] != 0) { //if linebreak true
				true_count_ += 1;
				false_count_ = 0;
			}
			else { //if linebreak false
				true_count_ = 0;
				false_count_ += 1;
			}

			if (true_count_ > debounce_iterations_){
				triggered_ = true;
			}
			else if (false_count_ > debounce_iterations_){
				triggered_ = false;
			}
		}

		Linebreak(std::string name, int debounce_iterations)
		{
			name_ = name;
			idx_ = std::numeric_limits<size_t>::max(); //bigger than any number. Will be this until we actually set it
			true_count_ = 0;
			false_count_ = 0;
			debounce_iterations_ = debounce_iterations;
			triggered_ = false;
		}
};





//create the class for the actionlib server
class IndexerAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::IndexerAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call controllers
		ros::ServiceClient indexer_controller_client_; //create a ros client to send requests to the controller

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

		//other variables
		int n_balls_;

		//config variables, with defaults
		double server_timeout_; //overall timeout for your server
		double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		IndexerAction(const std::string &name, double server_timeout, double wait_for_server_timeout) :
			as_(nh_, name, boost::bind(&IndexerAction::executeCB, this, _1), false),
			action_name_(name),
                        server_timeout_(server_timeout),
                        wait_for_server_timeout_(wait_for_server_timeout)
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		indexer_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command", false, service_connection_header);

		//read initial number of balls from config
		//TODO
		n_balls_ = 3;
	}

		~IndexerAction(void)
		{
		}

		//Use to make pauses while still checking timed_out_ and preempted_
		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out_ && ros::ok())
			{
				if(as_.isPreemptRequested() || !ros::ok())
				{
					preempted_ = true;
					ROS_ERROR_STREAM("indexer_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("indexer_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}


		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			//get index of linebreak sensors for this actionlib server
			static size_t linebreak_idx = std::numeric_limits<size_t>::max();
			if ((linebreak_idx >= joint_state.name.size()))
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == "cargo_intake_linebreak_1") //TODO: define this in the hardware interface
						linebreak_idx = i;
				}
			}

			//update linebreak counts based on the value of the linebreak sensor
			if (linebreak_idx < joint_state.position.size())
			{
				bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
				if(linebreak_true)
				{
					linebreak_true_count_ += 1;
				}
				else
				{
					linebreak_true_count_ = 0;
				}
			}
			else
			{
				ROS_WARN_THROTTLE(2.0, "intake_cargo_server : intake line break sensor not found in joint_states");
				linebreak_true_count_ = 0;
			}
		}
		}

		//function to send balls towards intake until linebreak right inside indexer is triggered - default state if less than 5 balls
		bool goToPositionIntake()
		{
			return true;
		}

		//function to send balls towards shooter until linebreak right before shooter is triggered - default state if have 5 balls
		bool goToPositionShoot()
		{
			return true;
		}


		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::IndexerGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;


			//wait for all controller servers we need
			if(! indexer_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find indexer_controller");
				as_.setPreempted();
				return;
			}


			//Three possible actions we can do:
			//0: go to position intake (back up until trigger linebreak sensor just inside the indexer)
			//1: intake a ball
			//2: feed one ball to the shooter, then stop at position shoot (go forward until trigger linebreak sensor just before shooter)

			switch(goal->action)
			{
				case 0: //go to position intake


					break;
				case 1: //intake a ball



					break;
				case 2: //feed a ball to the shooter



					break;
				default:
					ROS_ERROR_STREAM("Indexer server: " << goal->action << " is not a valid action (valid ones are 0,1,2)"
					preempted_ = true;

			}


			//Basic controller call ---------------------------------------
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("indexer_server: what this is doing");
				//call controller client, if failed set preempted_ = true, and log an error msg



				//if necessary, run a loop to wait for the controller to finish
				while(!preempted_ && !timed_out_ && ros::ok())
				{
					//check preempted_
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling ______ controller");
						preempted_ = true;
					}
					//test if succeeded, if so, break out of the loop
					else if(test here) {
						break;
					}
					//check timed out - TODO might want to use a timeout for this specific controller call rather than the whole server's timeout?
					else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
						ROS_ERROR_STREAM(action_name_ << ": timed out while calling ______ controller");
						timed_out_ = true;
					}
					//otherwise, pause then loop again
					else {
						r.sleep();
					}
				}
			}
			//preempt handling (skip this and set final state at end if only 1 possble final state)
			/*
			if(preempted_ || timed_out_ || !ros::ok())
			{}
			*/

			//if necessary, pause a bit between doing things (between piston firings usually)
			/* e.g.
			pause(sec_to_pause, "what we're pausing for");
			*/





			//Basic actionlib server call -------------------------------------
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("what this is doing");
				//Call actionlib server
				/* e.g.
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);
				*/
				//wait for actionlib server
				//e.g. waitForActionlibServer(ac_elevator_, 30, "calling elevator server"); //method defined below. Args: action client, timeout in sec, description of activity
			}
			//preempt handling or pause if necessary (see basic controller call)





			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that


			//log result and set actionlib server state appropriately
			behavior_actions::IndexerResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out_ = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out_ = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out_ = false;
				result.success = true;
				as_.setSucceeded(result);
			}

			return;

		}


		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
		{
			double request_time = ros::Time::now().toSec();

			//wait for actionlib server to finish
			std::string state;
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				state = action_client.getState().toString();

				if(state == "PREEMPTED") {
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server returned preempted_ during " << activity);
					preempted_ = true;
				}
				//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
				else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
						(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
				{
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
					break; //stop waiting
				}
				//checks related to this file's actionlib server
				else if (as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempted_ during " << activity);
					preempted_ = true;
				}
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else { //if didn't succeed and nothing went wrong, keep waiting
					ros::spinOnce();
					r.sleep();
				}
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "indexer_server");

	//get config values
	ros::NodeHandle n;

        double server_timeout = 10;
        double wait_for_server_timeout = 10;

	/* e.g.
	//ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params"); //node handle for a lower-down namespace

	if (!n.getParam("/teleop/teleop_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout_ in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");
	*/

	//create the actionlib server
	IndexerAction indexer_action("indexer_server", server_timeout, wait_for_server_timeout);

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
