#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/rate_limiter.h"
#include "teleop_joystick_control/SendZeroState.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

double rot_cmd = 0.0;
double x_cmd = 0.0;
double y_cmd = 0.0;

geometry_msgs::Twist combined_cmd_vel;

ros::Publisher combined_cmd_vel_pub;
ros::Publisher pid_enable_pub;

ros::ServiceClient BrakeSrv;

std::unique_ptr<rate_limiter::RateLimiter> rotation_rate_limiter;

teleop_joystick_control::TeleopJoystickCompConfig config;

void publishCombinedCmdVel(void)
{
	combined_cmd_vel_pub.publish(combined_cmd_vel);
}

void publishPIDEnable(void)
{
		std_msgs::Bool enable_pub_msg;
		enable_pub_msg.data = true;
		pid_enable_pub.publish(enable_pub_msg);
}

void sendZeroStateCallback(const teleop_joystick_control::SendZeroState::ConstPtr &send_zero_state_msg)
{
	if(send_zero_state_msg->sendRobotZero)
	{
		if(send_zero_state_msg->callBrakeSrv)
		{
			std_srvs::Empty empty;
			if (!BrakeSrv.call(empty))
			{
				ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
			}
			ROS_INFO("BrakeSrv called");
		}

		combined_cmd_vel.linear.x = 0.0;
		combined_cmd_vel.linear.y = 0.0;

		combined_cmd_vel.angular.z = 0.0;
	}
	else if(send_zero_state_msg->sendRotationZero)
	{
		combined_cmd_vel.linear.x = x_cmd;
		combined_cmd_vel.linear.y = y_cmd;

		combined_cmd_vel.angular.z = 0.0;
	}
	else if(send_zero_state_msg->sendRotationZero)
	{
		combined_cmd_vel.linear.x = 0.0;
		combined_cmd_vel.linear.y = 0.0;

		combined_cmd_vel.angular.z = rot_cmd;
	}
	else
	{
		combined_cmd_vel.linear.x = x_cmd;
		combined_cmd_vel.linear.y = y_cmd;

		combined_cmd_vel.angular.z = rot_cmd;
	}
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &teleop_msg)
{
	x_cmd = teleop_msg->linear.x;
	y_cmd = teleop_msg->linear.y;
}

void orientCallback(const std_msgs::Float64::ConstPtr &orient_msg)
{
	const double rotation = rotation_rate_limiter->applyLimit(orient_msg->data, ros::Time::now());
	rot_cmd = rotation;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "combine_orient_strafing_node");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");

	if(!n_params.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in orient_strafing");
	}
	if(!n_params.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in orient_strafing");
	}

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in orient_strafing_node");
	}

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompConfig> drw(n_params, config);

	rotation_rate_limiter = std::make_unique<rate_limiter::RateLimiter>(-config.max_rot, config.max_rot, config.drive_rate_limit_time);

	combined_cmd_vel.linear.x = 0.0;
	combined_cmd_vel.linear.y = 0.0;
	combined_cmd_vel.linear.z = 0.0;
	combined_cmd_vel.angular.x = 0.0;
	combined_cmd_vel.angular.y = 0.0;
	combined_cmd_vel.angular.z = 0.0;

	ros::Subscriber send_zero_sub = n.subscribe("send_zero_state", 5, sendZeroStateCallback);
	ros::Subscriber teleop_sub = n.subscribe("/teleop/swerve_drive_controller/cmd_vel", 5, teleopCallback);
	ros::Subscriber orient_sub = n.subscribe("control_effort", 5, orientCallback);

	combined_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 5);
	pid_enable_pub = n.advertise<std_msgs::Bool>("pid_enable", 1);

	ros::Rate r = 100;

	while(ros::ok())
	{
		publishPIDEnable();
		publishCombinedCmdVel();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
