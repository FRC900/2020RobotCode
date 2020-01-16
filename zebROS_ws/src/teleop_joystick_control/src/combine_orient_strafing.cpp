#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

bool enable_combination = false;

geometry_msgs::Twist combined_cmd_vel;

ros::Publisher combined_cmd_vel_pub;

void publishCombinedCmdVel(void)
{
	if(enable_combination)
		combined_cmd_vel_pub.publish(combined_cmd_vel);
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &teleop_msg)
{
	combined_cmd_vel.linear.x = teleop_msg->linear.x;
	combined_cmd_vel.linear.y = teleop_msg->linear.y;
}

void orientCallback(const geometry_msgs::Twist::ConstPtr &orient_msg)
{
	combined_cmd_vel.angular.z = orient_msg->angular.z;

	enable_combination = (orient_msg->angular.z != 0.0) ? true : false;
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

	combined_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("combine_orient_strafing/cmd_vel", 5);

	ros::Rate r = 100;

	while(ros::ok())
	{
		publishCombinedCmdVel();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
