#include <frc_msgs/JoystickState.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>

realtime_tools::RealtimeBuffer<geometry_msgs::Twist> teleop_cmd_vel;
realtime_tools::RealtimeBuffer<geometry_msgs::Twist> rotation_pid_cmd_vel;

ros::Publisher combined_cmd_vel_pub;

void combineData()
{
	geometry_msgs::Twist combined_cmd_vel;

	combined_cmd_vel.linear.x = teleop_cmd_vel.linear.x;
	combined_cmd_vel.linear.y = teleop_cmd_vel.linear.y;
	combined_cmd_vel.linear.z = teleop_cmd_vel.linear.z;

	combined_cmd_vel.angular.x = rotation_pid_cmd_vel.angular.x;
	combined_cmd_vel.angular.y = rotation_pid_cmd_vel.angular.y;
	combined_cmd_vel.angular.z = rotation_pid_cmd_vel.angular.z;

	combined_cmd_vel_pub.publish(combined_cmd_vel);
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &teleop_msg)
{
}

void rotationCallback(const geometry_msgs::Twist::ConstPtr &rotation_msg)
{
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "auto_rotate_strafing_node");
	ros::NodeHandle n;

	teleop_cmd_vel.linear.x = 0.0;
	teleop_cmd_vel.linear.y = 0.0;
	teleop_cmd_vel.linear.z = 0.0;

	teleop_cmd_vel.angular.x = 0.0;
	teleop_cmd_vel.angular.y = 0.0;
	teleop_cmd_vel.angular.z = 0.0;

	rotation_cmd_vel.linear.x = 0.0;
	rotation_cmd_vel.linear.y = 0.0;
	rotation_cmd_vel.linear.z = 0.0;

	rotation_cmd_vel.angular.x = 0.0;
	rotation_cmd_vel.angular.y = 0.0;
	rotation_cmd_vel.angular.z = 0.0;

	ros::Subscriber teleop_cmd_vel_sub = n.subscribe("swerve_drive_controller/cmd_vel", 5, teleopCallback);
	ros::Subscriber rotation_pid_cmd_vel_sub = n.subscribe("fill_in_later/cmd_vel", 5, rotationCallback);

	combined_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("auto_rotate_strafing/cmd_vel", 5);

	ros::Rate r(50);

	while(ros::ok())
	{
		combineData();

		r.sleep();
	}

	return 0;
}
