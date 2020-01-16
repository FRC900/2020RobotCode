#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "teleop_joystick_control/RobotOrient.h"

bool enable_combination = false;

geometry_msgs::Twist combined_cmd_vel;

ros::Publisher combined_cmd_vel_pub;

bool enableCombinationCallback(teleop_joystick_control::EnableRotationStrafingCombination::Request& req,
					teleop_joystick_control::EnableRotationStrafingCombination::Response&/* res*/)
{
	// Used to switch between robot orient and field orient driving
	robot_orient = req.robot_orient;
	offset_angle = req.offset_angle;
	ROS_WARN_STREAM("Robot Orient = " << (robot_orient) << ", Offset Angle = " << offset_angle);
	return true;
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &teleop_msg)
{
	combined_cmd_vel.linear.x = teleop_msg->linear.x;
	combined_cmd_vel.linear.y = teleop_msg->linear.y;

	if(enable_combination)
		combined_cmd_vel_pub.publish(combined_cmd_vel);
}

void orientCallback(const geometry_msgs::Twist::ConstPtr &orient_msg)
{
	combined_cmd_vel.angular.z = orient_msg->angular.z;

	if(enable_combination)
		combined_cmd_vel_pub.publish(combined_cmd_vel);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "combine_orient_strafing_node");
	ros::NodeHandle n;

	combined_cmd_vel.linear.x = 0.0;
	combined_cmd_vel.linear.y = 0.0;
	combined_cmd_vel.linear.z = 0.0;
	combined_cmd_vel.angular.x = 0.0;
	combined_cmd_vel.angular.y = 0.0;
	combined_cmd_vel.angular.z = 0.0;

	ros::Subscriber teleop_cmd_vel_sub = n.subscribe("swerve_drive_controller/cmd_vel", 5, teleopCallback);
	ros::Subscriber orient_pid_cmd_vel_sub = n.subscribe("orient_pid/orient_cmd", 5, orientCallback);

	ros::ServiceServer enable_combine_orient_strafing_service = n.advertiseService("enable_combine_orient_strafing_service", enableCombinationCallback);

	combined_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("combine_orient_strafing/cmd_vel", 5);

	ros::spin();

	return 0;
}
