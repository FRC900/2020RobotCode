#include <ros/ros.h>
#include <behaviors/linebreak.h>
#include <sensor_msgs/JointState.h>
#include <talon_state_msgs/TalonState.h>
#include <std_msgs/UInt8.h>

//linebreaks, need to initialize in main() after the node handle is created, but using shared pointers to make them global (so jointStateCallback() can access)
std::shared_ptr<Linebreak> intake_linebreak;
std::shared_ptr<Linebreak> indexer_front_linebreak;
std::shared_ptr<Linebreak> indexer_linebreak;
std::shared_ptr<Linebreak> shooter_linebreak;

//velocity variables, updated by talonStateCallback()
double intake_velocity = 0; //default to 0
double indexer_velocity = 0;


//subscriber callback functions

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	if(!intake_linebreak->update(joint_state)) {
		ROS_ERROR("Num balls publisher node failed to update intake linebreak");
	}
	if(!indexer_front_linebreak->update(joint_state)) {
		ROS_ERROR("Num balls publisher node failed to update indexer front linebreak");
	}
	if(!indexer_linebreak->update(joint_state)) {
		ROS_ERROR("Num balls publisher node failed to update indexer linebreak");
	}
	if(!shooter_linebreak->update(joint_state)) {
		ROS_ERROR("Num balls publisher node failed to update shooter linebreak");
	}
}

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
		intake_velocity = talon_state.speed[intake_idx];
		indexer_velocity = talon_state.speed[indexer_idx];
	}
}


int main(int argc, char **argv)
{
	//start the node
	ros::init(argc, argv, "num_powercells_publisher");
	ros::NodeHandle nh;

	//initialize linebreaks
	intake_linebreak = std::make_shared<Linebreak>("intake_linebreak");
	indexer_front_linebreak = std::make_shared<Linebreak>("indexer_front_linebreak");
	indexer_linebreak = std::make_shared<Linebreak>("indexer_linebreak");
	shooter_linebreak = std::make_shared<Linebreak>("shooter_linebreak");

	//subscribers
	ros::Subscriber joint_states_sub = nh.subscribe("/frcrobot_jetson/joint_states", 1, jointStateCallback);
	ros::Subscriber talon_states_sub = nh.subscribe("/frcrobot_jetson/talon_states", 1, talonStateCallback);

	//publishers
	ros::Publisher num_balls_pub = nh.advertise<std_msgs::UInt8>("num_powercells", 1);
	ros::Publisher num_indexer_balls_pub = nh.advertise<std_msgs::UInt8>("num_indexer_powercells", 1);
	ros::Publisher num_stored_balls_pub = nh.advertise<std_msgs::UInt8>("num_stored_powercells", 1);

	//storage variables for num balls
	int num_balls = 3; //default is 3
	int num_indexer_balls = 3;
	int num_stored_balls = 3;

	nh.getParam("/initial_num_powercells", num_balls);
	num_indexer_balls = num_balls; //assume all the initial balls are properly stored in the indexer
	num_stored_balls = num_balls;

	//loop and process logic
	ros::Rate r(20);
	double last_intake_velocity = 0;
	double last_indexer_velocity = 0;
	std_msgs::UInt8 msg; //will be reused a lot
	while(ros::ok())
	{
		//TODO check that positive velocity means forward

		//only do linebreak checks if this velocity is the same as last velocity - otherwise we can't guarantee the ball's direction of motion
		if((intake_velocity > 0 && last_intake_velocity > 0) || (intake_velocity < 0 && last_intake_velocity < 0)) {

			//intake linebreak checks
			if(intake_velocity > 0 && intake_linebreak->rising_edge_happened_) {
				num_balls++;
				intake_linebreak->resetPulseDetection();
			}
			else if(intake_velocity < 0 && intake_linebreak->falling_edge_happened_) {
				num_balls--;
				intake_linebreak->resetPulseDetection();
			}
		}

		if((indexer_velocity > 0 && last_indexer_velocity > 0) || (indexer_velocity < 0 && last_indexer_velocity < 0)) {

			//indexer front linebreak checks
			if(indexer_velocity > 0 && indexer_front_linebreak->rising_edge_happened_) {
				num_indexer_balls++;
				indexer_front_linebreak->resetPulseDetection();
			}
			if(indexer_velocity < 0 && indexer_front_linebreak->falling_edge_happened_) {
				num_indexer_balls--;
				indexer_front_linebreak->resetPulseDetection();
			}

			//indexer storage linebreak checks
			if(indexer_velocity > 0 && indexer_linebreak->rising_edge_happened_) {
				//NOTE: technically a pulse (rising then falling edge) while moving forward means properly stored, but can't detect that easily w/o assuming anything about velocity.
				//Testing for a rising edge while moving forwards means essentially the same thing according to testing, so using that here
				num_stored_balls++;
				indexer_linebreak->resetPulseDetection();
			}
			if(indexer_velocity < 0 && indexer_linebreak->falling_edge_happened_) {
				num_stored_balls--;
				indexer_linebreak->resetPulseDetection();
			}

			//shooter linebreak checks
			if(indexer_velocity > 0 && shooter_linebreak->falling_edge_happened_) {
				num_balls--;
				num_indexer_balls--;
				num_stored_balls--;
				shooter_linebreak->resetPulseDetection();
			}
		}

		last_intake_velocity = intake_velocity;
		last_indexer_velocity = indexer_velocity;


		//publish
		msg.data = num_balls;
		num_balls_pub.publish(msg);

		msg.data = num_indexer_balls;
		num_indexer_balls_pub.publish(msg);

		msg.data = num_stored_balls;
		num_stored_balls_pub.publish(msg);


		//reset linebreak detection so we know that any rising/falling edges happened after the last iteration of the loop
		intake_linebreak->resetPulseDetection();
		indexer_front_linebreak->resetPulseDetection();
		indexer_linebreak->resetPulseDetection();
		shooter_linebreak->resetPulseDetection();

		//ros stuff
		ros::spinOnce();
		r.sleep();
	}
}
