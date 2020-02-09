#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

#include "behaviors/AlignToShootAction.h"

class AlignToShootAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignToShootAction> as_;
		std::string action_name_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10};
		double start_time_;

		double server_timeout_; //overall timeout for your server
		double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one

	public:
		AlignToShootAction(const std::string &name, server_timeout, wait_for_server_timeout) :
			as_(nh_, name, boost::bind(&AlignToShoortAction::executeCB, this> _1), false),
			action_name_(name)
			server_timeout_(server_timeout),
			wait_for_server_timeout_(wait_for_server_timeout),
			ac_align_("/align/align_server", true), //TODO: Check name
			ac_shooter_("/shooter/shooter_server", true)
		{
			as_.start();

			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";
		}

		~AlignToShootAction(void)
		{
		}

		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out && ros::ok())
			{
				if(as_isPreemptRequested() || !ros::ok())
				{
					preempted_ = true;
					ROS_ERROR_STREAM("align_to_shoot_server: preempt during pause() - " << activity);
				}
				else if((ros::Time::now().toSec() - start_time_) >= server_timeout_)
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

		void executeCB(const behaviors::AlignToShootGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			if(!ac_align_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find align actionlib server");
				as_.setPreempted();
				return;
			}


			if(!preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("what this is doing");
			}


			behaviors::AlignToShootResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result_timed_out_ = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out_ = true;
				result.success = false;
				as_.setSucceeded(result);
			}
			else {
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
	ros::init(argc, argv, "align_then_shoot_server");

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
	AlignThenShootAction align_then_shoot_action("align_then_shoot_server", server_timeout, wait_for_server_timeout);

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
