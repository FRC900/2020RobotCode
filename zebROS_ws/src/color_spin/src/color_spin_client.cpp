#include "ros/ros.h"
#include "color_spin/color_algorithm.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_spin_algorithm");


	ros::NodeHandle nh;
	ros::ServiceClient client = nh.advertiseService("color_spin_algorithm", rotate);
	ROS_INFO("Ready to calculate rotation");
	ros::spin();

	return 0;

}

