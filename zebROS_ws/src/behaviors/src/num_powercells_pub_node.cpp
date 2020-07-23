#include <ros/ros.h>
#include <behaviors/linebreak.h>
#include <sensor_msgs/JointState.h>
#include <talon_state_msgs/TalonState.h>
#include <frc_msgs/MatchSpecificData.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <behavior_actions/resetBallSrv.h>

//velocity variables, updated by talonStateCallback()
double intake_percent_out = 0; //default to 0
double indexer_velocity = 0;

//storage variables for num balls
int num_balls = 3; //default is 3
int num_indexer_balls = 3;



//subscriber callback functions

void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
{
	//get intake and indexer velocities
	static size_t intake_idx = std::numeric_limits<size_t>::max();
	static size_t indexer_idx = std::numeric_limits<size_t>::max();

	//test if we don't know the correct index for both of them
	if (intake_idx >= talon_state.name.size() || indexer_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == "intake_joint")
			{
				intake_idx = i;
			}
			if (talon_state.name[i] == "indexer_joint")
			{
				indexer_idx = i;
			}
		}
		if (intake_idx >= talon_state.name.size()) {
			ROS_ERROR("Num balls publisher couldn't find intake_joint motor");
		}
		if (indexer_idx >= talon_state.name.size()) {
			ROS_ERROR("Num balls publisher couldn't find intake_joint motor");
		}
	}
	else
	{
		intake_percent_out = talon_state.set_point[intake_idx];
		indexer_velocity = talon_state.set_point[indexer_idx];
	}
}

//Checks whether or not the robot is disabled
bool disabled = true;

void matchDataCallback(const frc_msgs::MatchSpecificData & matchdata)
{
	disabled = matchdata.Disabled;
}

//Callback function to reset all ball counters
bool resetBall(behavior_actions::resetBallSrv::Request &req,
			   behavior_actions::resetBallSrv::Response &res)
{
	num_balls = req.n_balls;
	num_indexer_balls = req.n_balls;
	return true;
}

int main(int argc, char **argv)
{
	//start the node
	ros::init(argc, argv, "num_powercells_publisher");
	ros::NodeHandle nh;

	//initialize linebreaks
	Linebreak intake_linebreak("intake_linebreak");
	Linebreak indexer_linebreak("indexer_linebreak");
	Linebreak shooter_linebreak("shooter_linebreak");

	//subscribers
	ros::Subscriber talon_states_sub = nh.subscribe("/frcrobot_jetson/talon_states", 1, talonStateCallback);
	ros::Subscriber match_data_sub = nh.subscribe("/frcrobot_rio/match_data", 1, matchDataCallback);

	//publishers
	ros::Publisher num_balls_pub = nh.advertise<std_msgs::UInt8>("num_powercells", 1);
	ros::Publisher num_indexer_balls_pub = nh.advertise<std_msgs::UInt8>("num_indexer_powercells", 1);

	//reset all ball counts to 0
	ros::ServiceServer reset_ball_count = nh.advertiseService("reset_ball", resetBall);

	//checks to make sure the parameter read was successful
	if(!nh.getParam("/initial_num_powercells", num_balls))
	{
		ROS_ERROR("Couldn't read initial n balls in num powercell publisher");
		num_balls = 3;
	}
	num_indexer_balls = num_balls; //assume all the initial balls are properly stored in the indexer

	//loop and process logic
	ros::Rate r(20);
	double last_intake_percent_out = 0;
	double last_indexer_velocity = 0;
	std_msgs::UInt8 msg; //will be reused a lot
	while(ros::ok())
	{
		//TODO check that positive velocity means forward

		//only do linebreak checks if this velocity is the same as last velocity - otherwise we can't guarantee the ball's direction of motion
		if (disabled == false) {
			if((intake_percent_out > 0 && last_intake_percent_out > 0) || (intake_percent_out < 0 && last_intake_percent_out < 0)) {

				//intake linebreak checks
				if(intake_percent_out > 0 && intake_linebreak.rising_edge_happened_) {
					ROS_WARN_STREAM("Ball entered intake; percent out: " << intake_percent_out);
					num_balls++;
					intake_linebreak.resetPulseDetection();
				}
				else if(intake_percent_out < 0 && intake_linebreak.falling_edge_happened_) {
					ROS_WARN_STREAM("Ball exited intake; percent out: " << intake_percent_out);
					if(num_balls > 0) {
						num_balls--;
					}
					else {
						ROS_ERROR("Sensor detected ball being shot but number of balls was already at 0");
					}
					intake_linebreak.resetPulseDetection();
				}
			}

			if((indexer_velocity > 0 && last_indexer_velocity > 0) || (indexer_velocity < 0 && last_indexer_velocity < 0)) {

				//indexer storage linebreak checks
				if(indexer_velocity > 0 && indexer_linebreak.rising_edge_happened_) {
					//NOTE: technically a pulse (rising then falling edge) while moving forward means stored, but can't detect that easily w/o assuming anything about velocity.
					//Testing for a rising edge while moving forwards means essentially the same thing according to testing, so using that here
					ROS_WARN_STREAM("Ball entered indexer; speed setpoint: " << indexer_velocity);
					num_indexer_balls++;
					indexer_linebreak.resetPulseDetection();
				}
				if(indexer_velocity < 0 && indexer_linebreak.falling_edge_happened_) {
					ROS_WARN_STREAM("Ball exited indexer; speed setpoint: " << indexer_velocity);
					if(num_indexer_balls > 0) {
						num_indexer_balls--;
					}
					else {
						ROS_ERROR("Sensor detected ball being shot but number of balls was already at 0");
					}

					indexer_linebreak.resetPulseDetection();
				}

				//shooter linebreak checks
				if(indexer_velocity > 0 && shooter_linebreak.falling_edge_happened_) {
					ROS_WARN_STREAM("Ball left shooter; speed setpoint: " << indexer_velocity);
					if(num_balls > 0) {
						num_balls--;
					}
					else {
						ROS_ERROR("Sensor detected ball being shot but number of balls was already at 0");
					}
					if(num_indexer_balls > 0) {
						num_indexer_balls--;
					}
					else {
						ROS_ERROR("Sensor detected ball being shot but number of balls was already at 0");
					}
					shooter_linebreak.resetPulseDetection();
				}
			}
		}
		last_intake_percent_out = intake_percent_out;
		last_indexer_velocity = indexer_velocity;


		//publish
		msg.data = num_balls;
		num_balls_pub.publish(msg);

		msg.data = num_indexer_balls;
		num_indexer_balls_pub.publish(msg);



		//reset linebreak detection so we know that any rising/falling edges happened after the last iteration of the loop
		intake_linebreak.resetPulseDetection();
		indexer_linebreak.resetPulseDetection();
		shooter_linebreak.resetPulseDetection();

		//ros stuff
		ros::spinOnce();
		r.sleep();
	}
}
