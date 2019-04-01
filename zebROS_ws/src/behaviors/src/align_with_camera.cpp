#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
//tf stuff
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool publish = false;
bool publish_last = false;
bool goals_found = false;
tf2_ros::Buffer buffer;
std::string target_frame;
geometry_msgs::PointStamped relative_goal_location;

bool debug = true;

void cameraCB(const geometry_msgs::PointStampedConstPtr& raw_goal_location)
{
	goals_found = true;
	try
	{
		if(debug)
		{
			ROS_INFO_THROTTLE(1, " RAW point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
					raw_goal_location->point.x,
					raw_goal_location->point.y,
					raw_goal_location->point.z);
		}
		buffer.transform(*raw_goal_location, relative_goal_location, target_frame);
		if(debug)
		{
			ROS_INFO_THROTTLE(1, "RELATIVE point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
					relative_goal_location.point.x,
					relative_goal_location.point.y,
					relative_goal_location.point.z);
		}
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Failed %s\n", ex.what());
	}
}

bool startStopAlign(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish = req.data;
	res.success = true;

	return true;
}

void startStopCallback(std_msgs::Bool msg)
{
	publish = msg.data;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "align_with_camera");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "align_with_camera_params");
	ros::NodeHandle n_private_params("~");

	double last_command;
	double last_command_published = 0.0;

	//read configs
	double cmd_vel_to_pub;
	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_camera");
	double error_threshold;
	if(!n_params.getParam("error_threshold", error_threshold))
		ROS_ERROR_STREAM("Could not read error_threshold in align_with_camera");
	if(!n_private_params.getParam("target_frame", target_frame))
		ROS_ERROR_STREAM("Could not read target_frame in align_with_camera");

	//set up publisher for publish_pid_cmd_vel node
	ros::Publisher command_pub = n.advertise<std_msgs::Float64>("align_with_camera/command", 1);
	//set up feedback publisher for the align_server which uses this node
	ros::Publisher successful_y_align = n.advertise<std_msgs::Float64MultiArray>("align_with_camera/aligned", 1);
	//set up enable subscriber from align_server
	ros::Subscriber start_stop_sub = n.subscribe("align_with_camera/enable_pub", 1, &startStopCallback);
	//set up camera subscriber with transforms
	message_filters::Subscriber<geometry_msgs::PointStamped> camera_msg_sub(n, "pointstamped_goal_msg", 1);
	//advertise service to start or stop align (who uses this?)
	ros::ServiceServer start_stop_service = n.advertiseService("align_with_camera", startStopAlign);

	//set up transforms for camera -> mechanismt checkout align_with_zed -- ../../goal_detection/src/tr
	tf2_ros::TransformListener tf2(buffer);
	tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter(camera_msg_sub, buffer, target_frame, 10, 0);
	tf2_filter.registerCallback(cameraCB);

	std_msgs::Float64 cmd_msg;
	cmd_msg.data = 0;

	ros::Rate r(50);

	while(ros::ok())
	{
		bool aligned = false;
		double error = 0;
		
		if(target_frame == "panel_outtake")
		{
			error = relative_goal_location.point.y;
		}
		else if(target_frame == "cargo_outtake")
		{
			error = relative_goal_location.point.x;
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(0.5, "Unknown target_frame in align_with_camera");
		}

		if(fabs(error) < error_threshold)
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're aligned!! error = " << error);
			aligned = true;
			cmd_msg.data = 0;
		}
		else if(error > 0)
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're left. error = " << error);
			cmd_msg.data = -1*cmd_vel_to_pub;
		}
		else
		{
			if(debug)
				ROS_INFO_STREAM_THROTTLE(1, "we're right. error = " << error);
			cmd_msg.data = 1*cmd_vel_to_pub;
		}

		if(publish)
		{
			command_pub.publish(cmd_msg);
			last_command_published = cmd_msg.data;
		}
		else if(!publish && publish_last)
		{
			cmd_msg.data= 0;
			command_pub.publish(cmd_msg);
		}

		std_msgs::Float64MultiArray aligned_msg;
		aligned_msg.data.resize(5);
		aligned_msg.data[0] =  aligned ? 0.0 : std::numeric_limits<double>::max();
		successful_y_align.publish(aligned_msg);

		publish_last = publish;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
