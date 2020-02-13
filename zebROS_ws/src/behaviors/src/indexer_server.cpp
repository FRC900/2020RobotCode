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


int linebreak_debounce_iterations; //global so that main() can read it and the Linebreak class can use it

class Linebreak {
	public:
		std::string name_;
		size_t idx_; //index of this linebreak in the joint_states message
		int true_count_;
		int false_count_;
		int debounce_iterations_;
		bool triggered_;

		//called every time the joint state subscriber is run
		bool update(const sensor_msgs::JointState &joint_state)
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
				//if the index wasn't set, couldn't find it
				if ( idx_ >= joint_state.name.size() ) {
					ROS_ERROR_STREAM("Indexer server - Linebreak named " << name_ << " not found in joint_states");
					true_count_ = 0;
					false_count_ = 0;
					return false;
				}
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
			return true;
		}

		Linebreak(std::string name) //name as found in joint_states
		{
			name_ = name;
			idx_ = std::numeric_limits<size_t>::max(); //bigger than any number. Will be this until we actually set it
			true_count_ = 0;
			false_count_ = 0;
			debounce_iterations_ = linebreak_debounce_iterations; //read from global config value
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

		//subscribers
		ros::Subscriber joint_states_sub_;

		//linebreak sensors
		Linebreak indexer_linebreak_{"indexer_linebreak"}; //just inside the entrance to the indexer
		Linebreak shooter_linebreak_{"shooter_linebreak"}; //just before the shooter

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r_{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

		//other variables


		//Use to make pauses while still checking timed_out_ and preempted_
		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out_ && ros::ok())
			{
				checkPreemptedAndTimedOut("pausing during - " + activity);
				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}

		void checkPreemptedAndTimedOut(const std::string &activity)
		{
			if(as_.isPreemptRequested() || !ros::ok())
			{
				preempted_ = true;
				ROS_ERROR_STREAM("indexer_server: preempt during - " << activity);
			}
			else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
			{
				timed_out_ = true;
				ROS_ERROR_STREAM("indexer_server: timeout during - " << activity);
			}
		}


		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			//update all linebreak sensors
			if (! indexer_linebreak_.update(joint_state) ) {
				ROS_ERROR("indexer linebreak update failed, preempting indexer server");
				preempted_ = true;
			}
			if (! shooter_linebreak_.update(joint_state) ) {
				ROS_ERROR("shooter linebreak update failed, preempting indexer server");
				preempted_ = true;
			}
		}

		//function to send balls towards intake until linebreak right inside indexer is triggered - default state if less than 5 balls
		bool goToPositionIntake()
		{
			if (n_balls_ > 0 && !indexer_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok()){
				//set velocity to reverse
				controllers_2020_msgs::IndexerSrv srv;
				srv.request.indexer_velocity = -indexer_speed_; //TODO make sure negative means backward
				if ( !indexer_controller_client_.call(srv) )
				{
					ROS_ERROR("Indexer controller failed in indexer server, in goToPositionIntake()");
					preempted_ = true;
				}

				//keep going backwards until indexer linebreak is triggered (until ball right in front of intake)
				while (!indexer_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok())
				{
					checkPreemptedAndTimedOut("going to position intake");
					if(!preempted_ && !timed_out_){
						r_.sleep(); //sleep if we didn't preempt or timeout
					}
				}

				//no matter what happened, stop the indexer now
				srv.request.indexer_velocity = 0;
				indexer_controller_client_.call(srv);
			}
			if(preempted_ || timed_out_){
				return false;
			}
			return true;
		}

		//function to send balls towards shooter until linebreak right before shooter is triggered - default state if have 5 balls
		bool goToPositionShoot()
		{
			if (n_balls_ > 0 && !shooter_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok()){
				//set velocity to reverse
				controllers_2020_msgs::IndexerSrv srv;
				srv.request.indexer_velocity = indexer_speed_; //TODO make sure positive means forward
				if ( !indexer_controller_client_.call(srv) )
				{
					ROS_ERROR("Indexer controller failed in indexer server, in goToPositionShoot()");
					preempted_ = true;
				}

				//keep going forwards until shooter linebreak is triggered (until ball right in front of shooter)
				while (!shooter_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok())
				{
					checkPreemptedAndTimedOut("going to position shoot");
					if(!preempted_ && !timed_out_){
						r_.sleep(); //sleep if we didn't preempt or timeout
					}
				}

				//no matter what happened, stop the indexer now
				srv.request.indexer_velocity = 0;
				indexer_controller_client_.call(srv);
			}
			if(preempted_ || timed_out_){
				return false;
			}
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
                                    ROS_INFO_STREAM("Going to position intake in indexer actionlib server");
					if(!preempted_ && !timed_out_ && ros::ok())
					{
						goToPositionIntake();
					}

					break;
				case 1: //intake a ball
                                    ROS_INFO_STREAM("Intaking a ball in indexer actionlib server");



					break;
				case 2: //feed a ball to the shooter
                                    ROS_INFO_STREAM("Feeding a ball to the shooter in indexer actionlib server");



					break;
				default:
					ROS_ERROR_STREAM("Indexer server: invalid goal. " << goal->action << " is not a valid action (valid ones are 0,1,2)");
					preempted_ = true;
					break;
			}

/*
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






			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that


			//log result and set actionlib server state appropriately
			behavior_actions::IndexerResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out = false;
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
					r_.sleep();
				}
			}
		}

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		IndexerAction(const std::string &name) :
			as_(nh_, name, boost::bind(&IndexerAction::executeCB, this, _1), false),
			action_name_(name)
		{
			as_.start(); //start the actionlib server

			//do networking stuff
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			//initialize clients used to call controllers
			indexer_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command", false, service_connection_header);

			//initialize subscribers
			joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &IndexerAction::jointStateCallback, this);
		}

		~IndexerAction(void)
		{
		}

		//config variables
		double server_timeout_; //overall timeout for your server
		double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one
		int n_balls_;
		double indexer_speed_;

};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "indexer_server");
	ros::NodeHandle nh;

	//create the actionlib server
	IndexerAction indexer_action("indexer_server");

	//get config values
	ros::NodeHandle nh_indexer(nh, "actionlib_indexer_params");

	if (! nh.getParam("/actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations) ){
		ROS_ERROR("Couldn't read linebreak_debounce_iterations in indexer server");
		linebreak_debounce_iterations = 10;
	}
	if (! nh_indexer.getParam("server_timeout", indexer_action.server_timeout_) ){
		ROS_ERROR("Couldn't read server_timeout in indexer server");
		indexer_action.server_timeout_ = 10;
	}
	if (! nh_indexer.getParam("wait_for_server_timeout", indexer_action.wait_for_server_timeout_) ){
		ROS_ERROR("Couldn't read wait_for_server_timeout in indexer server");
		indexer_action.wait_for_server_timeout_ = 10;
	}
	if (! nh_indexer.getParam("initial_n_balls", indexer_action.n_balls_) ){
		ROS_ERROR("Couldn't read initial_n_balls in indexer server");
		indexer_action.n_balls_ = 0;
	}
	if (! nh_indexer.getParam("indexer_speed", indexer_action.indexer_speed_) ){
		ROS_ERROR("Couldn't read indexer_speed in indexer server");
		indexer_action.wait_for_server_timeout_ = 4; //TODO fix
	}


	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
