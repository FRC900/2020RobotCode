#include <ros/ros.h>
#include <pure_pursuit/PurePursuitAction.h>
#include <pure_pursuit/PurePursuitGoal.h>
#include <pure_pursuit/PurePursuitResult.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "pure_pursuit/spline.h"
#include <std_srvs/Trigger.h>
#include "base_trajectory/GenerateSpline.h"
#include <swerve_point_generator/FullGenCoefs.h>
#include <pure_pursuit/Point.h>
#include "tf2/LinearMath/Quaternion.h"

std::shared_ptr<actionlib::SimpleActionClient<pure_pursuit::PurePursuitAction>> ac;
ros::ServiceClient spline_gen;

bool trigger_pathing_cb(pure_pursuit::Point::Request &req, pure_pursuit::Point::Response &res)
{
	/*** GET SPLINE COEFFICIENTS**/
	ROS_ERROR_STREAM("Calling base_trajectory");
	base_trajectory::GenerateSpline srvBaseTrajectory;
	int point_num = req.points.size();
	srvBaseTrajectory.request.points.resize(point_num);
	ROS_INFO_STREAM(req.points[0].x << " " << req.points[0].y << " " << req.points[0].z);

	for(int i = 0; i<point_num; i++)
	{
		//x-movement
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].x);
		//y-movement
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].y);
		//z-rotation
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].z);
		//time for profile to run
		srvBaseTrajectory.request.points[i].time_from_start = ros::Duration(10*(i+1));
	}

	if(!spline_gen.call(srvBaseTrajectory))
	{
		ROS_ERROR_STREAM("spline_gen died");
	}

	nav_msgs::Path path = srvBaseTrajectory.response.path;

	/*** SEND GOAL TO ACTION SERVER ***/
	ROS_ERROR_STREAM("Sending goal to action server in pure pursuit test client");
    pure_pursuit::PurePursuitGoal goal;
    goal.path = path;
    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(15));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO_STREAM("state = " << state.toString());
    }
    else
        ROS_INFO_STREAM("timed out in pure_pursuit service");

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    ros::NodeHandle n;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = 1;
    ac = std::make_shared<actionlib::SimpleActionClient<pure_pursuit::PurePursuitAction>>("pure_pursuit_server", true);
    ros::ServiceServer trigger_pathing = n.advertiseService("pure_pursuit_service", &trigger_pathing_cb);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/path_to_goal/base_trajectory/spline_gen", false, service_connection_header);

    ac->waitForServer();

    ros::spin();

    return 0;
}
