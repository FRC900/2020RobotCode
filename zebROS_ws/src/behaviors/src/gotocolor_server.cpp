//THIS IS A TEMPLATE FILE. TO USE, COPY IT AND REPLACE:
// ServerName	with your server's name, e.g. CargoIntake
// server_name	with your server's name, e.g. cargo_intake
// Thing		with the name of your action file (if file is Intake.action, replace w/ Intake)


#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>
#include <cctype>

//include action files - for this actionlib server and any it sends requests to
#include "behaviors/GoToColorAction.h"

//include controller service files and other service files
// e.g. #include "controller_package/ControllerSrv.h"
// e.g. #include "sensor_msgs/JointState.h" -has linebreak sensor data FYI
#include "frc_msgs/MatchSpecificData.h"
#include "color_spin/color_algorithm.h"
#include "controllers_2020_msgs/ControlPanelSrv.h"
#include "controllers_2020_msgs/ClimberSrv.h"
#include "geometry_msgs/Twist.h"

//create the class for the actionlib server
class GoToColorControlPanelAction {
	protected:
		ros::NodeHandle nh_;
        ros::Subscriber match_sub_;
		ros::Subscriber color_sensor_sub_;
		actionlib::SimpleActionServer<behaviors::GoToColorAction> as_; //create the actionlib server
		std::string action_name_;
		string goal_color_;
		string current_color_;

		ros::ServiceClient color_algorithm_client_;

		ros::Publisher cmd_vel_publisher_;

		//clients to call other actionlib servers
		//e.g. actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		//clients to call controllers
		//e.g. ros::ServiceClient mech_controller_client_; //create a ros client to send requests to the controller
		ros::ServiceClient control_panel_controller_client_;
		ros::ServiceClient climber_controller_client_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

		double climb_wait;

		double drive_forward_speed;
		std::atomic<double> cmd_vel_forward_speed_;
                std::atomic<bool> stopped_;

                //config variables, with defaults
                double server_timeout_; //overall timeout for your server
                double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		GoToColorControlPanelAction(const std::string &name, server_timeout, wait_for_server_timeout) :
			as_(nh_, name, boost::bind(&GoToColorControlPanelAction::executeCB, this, _1), false),
			action_name_(name),
                        server_timeout_(server_timeout),
                        wait_for_server_timeout_(wait_for_server_timeout)
			//ac_elevator_("/elevator/elevator_server", true) example how to initialize other action clients, don't forget to add a comma on the previous line
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		//e.g. mech_controller_client_ = nh_.serviceClient<controller_package::ControllerSrv>("name_of_service", false, service_connection_header);

		match_sub_=nh.subscribe("/frcrobot_rio/match_data", 1000, matchColorCallback);
		color_sensor_sub_=nh.subscribe("/color_sensor" /*insert topic name*/, 1000, sensorColorCallback);
		
		color_algorithm_client_=nh.serviceClient<color_spin::color_algorithm>("color_spin_algorithm", false, service_connection_header);
		control_panel_controller_client_=nh.serviceClient<controllers_2020_msgs::ControlPanelSrv>("control_panel_controller", false, service_connection_header);
		climber_controller_client_=nh.serviceClient<controllers_2020_msgs::ClimberSrv>("climber_controller", false, service_connection_header);
		
		//initialize the publisher used to send messages to the drive base
                cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);	
	}

		~GoToColorControlPanelAction (void)
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
					ROS_ERROR_STREAM("server_name_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("server_name_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}
		
		// Basic thread which spams cmd_vel to the drive base to
                // continually drive forward during the climb
                void cmdVelThread()
                {
                        ROS_INFO_STREAM("the callback is being called");
                        geometry_msgs::Twist cmd_vel_msg;
                        stopped_ = false;

                        ros::Rate r(20);

                        while(ros::ok() && !stopped_)
                        {
                                cmd_vel_msg.linear.x = cmd_vel_forward_speed_;
                                cmd_vel_msg.linear.y = 0.0;
                                cmd_vel_msg.linear.z = 0.0;
                                cmd_vel_msg.angular.x = 0.0;
                                cmd_vel_msg.angular.y = 0.0;
                                cmd_vel_msg.angular.z = 0.0;

                                cmd_vel_publisher_.publish(cmd_vel_msg);
                                r.sleep();
                        }
                }

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::GoToColorGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			std::thread cmdVelThread(std::bind(&GoToColorControlPanelAction::cmdVelThread, this));
			cmd_vel_foward_speed_ = 0;
			//wait for all actionlib servers we need
			/* e.g.
			if(!ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find elevator actionlib server");
				as_.setPreempted();
				return;
			}
			*/

			//wait for all controller servers we need
			/* e.g.
			if(! mech_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find mech_controller");
				as_.setPreempted();
				return;
			}
			*/


			if(! control_panel_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find control_panel_controller");
				as_.setPreempted();
				return;
			}

			if(! climber_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find climber_controller");
				as_.setPreempted();
				return;
			}

			//Extend the climber to deploy the mechanism
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				controller_2020_msgs::ClimberSrv climb_srv;
				climb_srv.request.//???;
				if(!climber_controller_client_.call(climb_srv))
				{
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling climb service");
				}
			}				

			pause(climb_wait,"Extending climber");

			double panel_rotations;

			if(!preempted_ && !timed_out_ && ros::ok())
				cmd_vel_forward_speed_ = drive_forward_speed; //Drive forward while we rotate
			
			//Loop while calling rotation
			while(!preempted_ && !timed_out_ && ros::ok() && goal_color_ != current_color_)
			{
				color_spin::color_algorithm spin_srv;
				spin_srv.request.sensor_color = current_color_;
				spin_srv.request.fms_color = goal_color_;
				if(color_algorithm_client_.call(spin_srv))
				{
					panel_rotations = srv.response.rotate;
				} else {
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling color algorithm");
					preempted_ = true;
				}
					
				controller_2020_msgs::ControlPanelSrv panel_srv;
				if(!preempted_ && !timed_out_ && ros::ok())
				{
					panel_srv.request. = ;
					if(!control_panel_controller_client_.call(panel_srv)) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling control panel controller");
						preempted_ = true;
					}
				}
				
				//check preempted_
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling control panel controller");
					preempted_ = true;
				}
				//check timed out - TODO might want to use a timeout for this specific controller call rather than the whole server's timeout?
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out while calling control panel controller");
					timed_out_ = true;
				}
				//otherwise, pause then loop again
				else {
					r.sleep();
				}
			}

			
			stopped_ = true; //Stop driving forward
			
			//Retract the climber
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				controller_2020_msgs::ClimberSrv climb_srv;
				climb_srv.request.//???;
				if(!climber_controller_client_.call(climb_srv))
				{
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling climb service");
				}
			}				

			pause(climb_wait,"Retracting climber");

			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that


			//log result and set actionlib server state appropriately
			behaviors::GoToColorResult result;

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
					r.sleep();
				}
			}
		}

		void matchColorCallback(const frc_msgs::MatchSpecificData &match_data){
			goal_color_= std::toupper(match_data.controlPanelColor);
		}

		void sensorColorCallback(const /*insert message type here*/ &color_data){
			current_color_ = std::toupper(color_data./*insert name of data*/);
		}
};
int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "server_name_server");

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

	if (!n.getParam("/actionlib_gotocolor_params/drive_forward_speed", drive_forward_speed))
        {
                ROS_ERROR("Could not read drive_forward_speed in go_to_color_control_panel_server");
                drive_forward_speed = 0.2;
        }


	if (!n.getParam("/actionlib_gotocolor_params/climb_wait_time", climb_wait))
        {
                ROS_ERROR("Could not read climb_wait_time in go_to_color_control_panel_server");
                climb_wait = 1;
        }

	//create the actionlib server
	GoToColorControlPanelAction go_to_color_control_panel("go_to_color_control_panel", server_timeout, wait_for_server_timeout);

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
