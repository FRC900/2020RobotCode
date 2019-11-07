/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 * 4. transform goal point to vehicle coordinates and drive to that position
 */
#include "pure_pursuit/pure_pursuit.h"

bool PurePursuit::setup()
{
	// TODO (KCJ) - move these reads to the calling code, pass in just the values
	// No reason for this class to know anything about ROS
	// Read config values
	nh_.getParam("/frcrobot_jetson/pure_pursuit/lookahead_distance", lookahead_distance_);
	nh_.getParam("/frcrobot_jetson/swerve_drive_controller/max_speed", max_velocity_);
	nh_.getParam("/frcrobot_jetson/swerve_drive_controller/max_accel", max_accel_);
	nh_.getParam("/frcrobot_jetson/pure_pursuit/pos_tol", pos_tol_);
	nh_.getParam("/frcrobot_jetson/pure_pursuit/final_pos_tol", final_pos_tol_);

	if(!pid_controller_.init(ros::NodeHandle(nh_, "pure_pursuit/pid_parameters")))
	{
		ROS_ERROR_STREAM("PID controller failed to initiate.");
		return false;
	}

	time_of_last_cycle_ = ros::Time::now();
	pid_controller_.reset();

	return true;
}

// TODO : for large function parameters, making them const T & is more efficient
void PurePursuit::loadPath(nav_msgs::Path path)
{
	path_ = path;
	num_waypoints_ = path_.poses.size();
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a Twist (or just x,y
// velocity) and whoever called run would be responsible for sending that
// where it needs to go.
geometry_msgs::Twist PurePursuit::run(nav_msgs::Odometry odom)
{
	ROS_INFO_STREAM("----------------------------------------------");
	ROS_INFO_STREAM("current_position = " << odom.pose.pose.position.x
			<< " " << odom.pose.pose.position.y);

	geometry_msgs::PoseStamped next_waypoint;

	// Find point in path closest to odometry reading
	// TODO (KCJ) - another possibility here is looking to see which segment between two wayponints
	// the current position is normal to.  That is, if it is off track, assume it is off track to the left
	// or right of the desired path, and check to see which segment it is on if the point were projected
	// perpendicular back to the correct track.
	// See e.g. https://stackoverflow.com/questions/17581738/check-if-a-point-projected-on-a-line-segment-is-not-outside-it
	// It could hit multiple segments, though, so maybe the minimum distance of segments it is normal to, using
	// the right angle distance to the closest point along each segment
	// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
	// This would also potentially give a location between two segments as the current
	// location, which makes the next lookahead point also somewhere
	// between two waypoints.
	double minimum_distance = std::numeric_limits<double>::max();
	size_t minimum_idx = 0;
	for(int i = 0; i < num_waypoints_; i++)
	{
		ROS_INFO_STREAM("waypoint " << i << " = " << path_.poses[i].pose.position.x << ", " << path_.poses[i].pose.position.y);
		ROS_INFO_STREAM("distance from waypoint " << i << " = " << hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y));
		if(hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y) < minimum_distance)
		{
			minimum_distance = hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y);
			minimum_idx = i;
		}
	}
	ROS_INFO_STREAM("minimum_distance = " << minimum_distance);

	next_waypoint = path_.poses[std::min(num_waypoints_ - 1, minimum_idx+1)];

	ROS_INFO_STREAM("x-error: " << fabs(odom.pose.pose.position.x - next_waypoint.pose.position.x) << " y-error: " << fabs(odom.pose.pose.position.y - next_waypoint.pose.position.y) << " final_pos_tol: " << final_pos_tol_);
	if(minimum_idx == num_waypoints_ - 1 && fabs(odom.pose.pose.position.x - path_.poses[num_waypoints_ - 1].pose.position.x) < final_pos_tol_ && fabs(odom.pose.pose.position.y - path_.poses[num_waypoints_ - 1].pose.position.y) < final_pos_tol_)
	{
		// TODO : no reason for cmd_vel_ to be a member var, it can
		// be a local instead.
		cmd_vel_.linear.x = 0;
		cmd_vel_.linear.y = 0;
		cmd_vel_.linear.z = 0;
		cmd_vel_.angular.x = 0;
		cmd_vel_.angular.y = 0;
		cmd_vel_.angular.z = 0;

		return cmd_vel_;
	}

	ROS_INFO_STREAM("next_waypoint = " << next_waypoint.pose.position.x
			<< " " << next_waypoint.pose.position.y
			<< " " << next_waypoint.pose.position.z);

	ros::Duration dt = ros::Time::now() - time_of_last_cycle_;
	time_of_last_cycle_ = ros::Time::now();
	double roll, pitch, yaw, target_yaw, actual_yaw;
	tf::Quaternion odom_q(
			odom.pose.pose.orientation.w,
			odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z);
	tf::Quaternion waypoint_q(
			next_waypoint.pose.orientation.w,
			next_waypoint.pose.orientation.x,
			next_waypoint.pose.orientation.y,
			next_waypoint.pose.orientation.z);
	tf::Matrix3x3(odom_q).getRPY(actual_yaw, pitch, yaw);
	ROS_INFO_STREAM("waypoint quaternion = " << 
			next_waypoint.pose.orientation.w << " " <<
			next_waypoint.pose.orientation.x << " " <<
			next_waypoint.pose.orientation.y << " " <<
			next_waypoint.pose.orientation.z);
	tf::Matrix3x3(waypoint_q).getRPY(target_yaw, pitch, yaw);
	ROS_INFO_STREAM("roll + pitch = " << roll << " " << pitch);
	ROS_INFO_STREAM("yaw of robot = " << actual_yaw);
	ROS_INFO_STREAM("target yaw of robot = " << target_yaw);

	// Set the angle of the velocity
	geometry_msgs::Point32 base_link_waypoint;
	base_link_waypoint.x = next_waypoint.pose.position.x - odom.pose.pose.position.x;
	base_link_waypoint.y = next_waypoint.pose.position.y - odom.pose.pose.position.y;
	double mag = hypot(base_link_waypoint.x, base_link_waypoint.y);
	ROS_INFO_STREAM("distance to drive to next waypoint = " << mag);
	ROS_INFO_STREAM("max_velocity = " << max_velocity_);
	ROS_INFO_STREAM("distance to drive x = " << base_link_waypoint.x << "; distance to drive y = " << base_link_waypoint.y);
	cmd_vel_.linear.x = max_velocity_ * base_link_waypoint.x / mag;
	cmd_vel_.linear.y = max_velocity_ * base_link_waypoint.y / mag;
	cmd_vel_.linear.z = 0;
	cmd_vel_.angular.x = 0;
	cmd_vel_.angular.y = 0;
	cmd_vel_.angular.z = pid_controller_.updatePid(actual_yaw - target_yaw, dt);

	return cmd_vel_;
}
