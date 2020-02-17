#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <mutex>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/AlignToShootAction.h"

//include controller service files and other service files
#include "controllers_2020_msgs/TurretSrv.h"

#include <field_obj/Detection.h>
#include <talon_state_msgs/TalonState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>

//create the class for the actionlib server
class AlignToShootAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::AlignToShootAction> as_; //create the actionlib server
		std::string action_name_;

		//Subscriber
		ros::Subscriber goal_detection_data_sub_;
		std::mutex goal_msg_mutex_;
		field_obj::Detection goal_msg_;

		ros::Subscriber talon_states_sub_;
		std::atomic<double> cur_turret_position_;

		ros::Subscriber robot_heading_sub_;
		std::atomic<double> robot_yaw_;

		//clients to call controllers
		ros::ServiceClient turret_controller_client_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		bool aligned_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		AlignToShootAction(const std::string &name) :
			as_(nh_, name, boost::bind(&AlignToShootAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		goal_detection_data_sub_ = nh_.subscribe("/goal_detection/goal_detect_msg", 1, &AlignToShootAction::goalDetectionCallback, this);
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &AlignToShootAction::talonStateCallback, this);
		robot_heading_sub_ = nh_.subscribe("/imu/data", 1, &AlignToShootAction::robotHeadingCallback, this);

		//initialize client used to call controllers
		//e.g. mech_controller_client_ = nh_.serviceClient<controller_package::ControllerSrv>("name_of_service", false, service_connection_header);
		turret_controller_client_ = nh_.serviceClient<controllers_2020_msgs::TurretSrv>("/frcrobot_jetson/turret_controller/turret_command", false, service_connection_header);
	}

		~AlignToShootAction(void)
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
					ROS_ERROR_STREAM("align_to_shoot_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("align_to_shoot_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}

		void goalDetectionCallback(const field_obj::Detection &msg)
		{
			std::lock_guard<std::mutex> l(goal_msg_mutex_);
			goal_msg_ = msg;
		}

		void robotHeadingCallback(const sensor_msgs::Imu &msg)
		{
			const tf2::Quaternion imuQuat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
			double roll;
			double pitch;
			double yaw;
			tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

			if (yaw == yaw) // ignore NaN results
				robot_yaw_ = yaw;
		}

		void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
		{
			static size_t turret_idx = std::numeric_limits<size_t>::max();
			if (turret_idx >= talon_state.name.size())
			{
				for (size_t i = 0; i < talon_state.name.size(); i++)
				{
					if (talon_state.name[i] == "turret_joint") //TODO
					{
						turret_idx = i;
						break;
					}
				}
			}
			else {
				cur_turret_position_ = talon_state.position[turret_idx];
			}
		}

		double getTurretPosition()
		{
			//use goal_msg and angle of robot to determine turret position
			return 2;
		}

		void executeCB(const behavior_actions::AlignToShootGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;
			aligned_ = false;

			// Wait for turret controller to exist
			if(!turret_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{

				ROS_ERROR_STREAM(action_name_ << " can't find turret_controller");
				as_.setPreempted();
				return;
			}

			// Try over and over again to get the turret to its desired position (update position each time)
			while(!aligned_ && !preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO_STREAM(action_name_ << ": calling turret controller");
				//call controller client, if failed set preempted_ = true, and log an error msg
				controllers_2020_msgs::TurretSrv srv;
				srv.request.set_point = getTurretPosition();
				if(!turret_controller_client_.call(srv))
				{
					ROS_ERROR_STREAM("Failed to call turret_controller_client_ in AlignToShootAction");
					as_.setPreempted();
				}

				double start_turret_time = ros::Time::now().toSec();
				bool turret_timed_out = false; //This determines if this service call timed out, not if the entire server timed out
				//if necessary, run a loop to wait for the controller to finish
				while(!preempted_ && !timed_out_ && !turret_timed_out && ros::ok())
				{
					//check preempted_
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling turret controller");
						preempted_ = true;
					}
					//test if succeeded, if so, break out of the loop
					else if(cur_turret_position_ - srv.request.set_point < max_turret_position_error_) {
						break;
						aligned_ = true;
					}
					//check timed out 
					else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
						ROS_ERROR_STREAM(action_name_ << ": timed out while calling turret controller");
						timed_out_ = true;
					}
					else if((ros::Time::now().toSec() - start_turret_time > turn_turret_timeout_))
					{
						ROS_WARN_STREAM(action_name_ << ": turret controller timed out; running again");
					}
					//otherwise, pause then loop again
					else {
						r.sleep();
					}
				}
			}

			//log result and set actionlib server state appropriately
			behavior_actions::AlignToShootResult result;

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
					r.sleep();
				}
			}
		}

		//config variables, with defaults
        double server_timeout_; //overall timeout for your server
        double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one
        double turn_turret_timeout_; //timeout for waiting for turret to go to position 
		double max_turret_position_error_;

};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "align_to_shoot_server");

	//get config values
	ros::NodeHandle n;

	//create the actionlib server
	AlignToShootAction align_to_shoot_action("align_to_shoot_server");

	ros::NodeHandle n_params_align(n, "actionlib_align_params"); //node handle for a lower-down namespace
	if (!n_params_align.getParam("server_timeout", align_to_shoot_action.server_timeout_))
	{
		ROS_ERROR("Could not read server_timeout in align_server");
		align_to_shoot_action.server_timeout_ = 10;
	}
	if (!n_params_align.getParam("wait_for_server_timeout", align_to_shoot_action.wait_for_server_timeout_))
	{
		ROS_ERROR("Could not read wait_for_server_timeout in align_server");
		align_to_shoot_action.wait_for_server_timeout_ = 10;
	}
	if (!n_params_align.getParam("turn_turret_timeout", align_to_shoot_action.turn_turret_timeout_))
	{
		ROS_ERROR("Could not read turn_turret_timeout in align_server");
		align_to_shoot_action.turn_turret_timeout_ = 10;
	}
	if (!n_params_align.getParam("max_turret_position_error", align_to_shoot_action.max_turret_position_error_))
	{
		ROS_ERROR("Could not read max_turret_position_error in align_server");
		align_to_shoot_action.max_turret_position_error_ = 10;
	}

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
