#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <base_trajectory/GenerateSpline.h>
#include <pure_pursuit/PathAction.h>
#include <pure_pursuit/PathGoal.h>
#include <pure_pursuit/axis_state.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

class PathAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<pure_pursuit::PathAction> as_;
		std::string action_name_;

		ros::ServiceClient spline_gen_cli_;

		ros::Subscriber odom_sub_;
		nav_msgs::Odometry odom_;

		std::map<std::string, AlignActionAxisState> axis_states_;

		bool debug_;

	public:
		PathAction(const std::string &name, ros::NodeHandle nh)
			: nh_(nh)
			, as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false)
			, action_name_(name)
			, debug_(false) // TODO - config item?
		{
			as_.start();

			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			// TODO - not sure which namespace base_trajectory should go in
			spline_gen_cli_ = nh_.serviceClient<base_trajectory::GenerateSpline>("/path_to_goal/base_trajectory/spline_gen", false, service_connection_header);

			odom_sub_ = nh_.subscribe("some odometry topic", 1, &PathAction::odomCallback, this);
		}

		~PathAction(void) {}

		void odomCallback(const nav_msgs::Odometry &odom_msg)
		{
			odom_ = odom_msg;
		}

		bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			// TODO - give defaults so these aren't random values if getParam fails
			double timeout;
			if (!nh_.getParam(axis_config.timeout_param_, timeout))
			{
				ROS_ERROR_STREAM("Could not read param "
								 << axis_config.timeout_param_
								 << " in align_server");
				//return false;
			}
			double error_threshold;
			if (!nh_.getParam(axis_config.error_threshold_param_, error_threshold))
			{
				ROS_ERROR_STREAM("Could not read param "
								 << axis_config.error_threshold_param_
								 << " in align_server");
				//return false;
			}

			axis_states_.emplace(std::make_pair(axis_config.name_,
												AlignActionAxisState(axis_config.name_,
														nh_,
														axis_config.enable_pub_topic_,
														axis_config.command_pub_topic_,
														axis_config.state_pub_topic_,
														axis_config.error_sub_topic_,
														boost::bind(&PathAction::error_term_cb, this, _1, axis_config.name_),
														timeout,
														error_threshold)));
			return true;
		}

		// Callback for error term from PID node.  Compares error
		// reported from PID vs. threshhold for the given axis and
		// sets both the saved error as well as the aligned flag for that axis
		void error_term_cb(const std_msgs::Float64MultiArrayConstPtr &msg, const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in error_term_cb");
				return;
			}
			auto &axis = axis_it->second;
			//Check if error less than threshold
			axis.aligned_ = fabs(msg->data[0]) < axis.error_threshold_;
			axis.error_ = msg->data[0];
			if (debug_)
				ROS_WARN_STREAM_THROTTLE(1, name << " error: " << axis.error_ << " aligned: " << axis.aligned_);
		}

		void executeCB(const pure_pursuit::PathGoalConstPtr &goal)
		{
			// First, make the spline that represents the points we should travel to
			base_trajectory::GenerateSpline spline_gen_srv;
			const size_t point_num = goal->points.size();
			spline_gen_srv.request.points.resize(point_num);

			ros::Rate r(20); // TODO : I should be a config item

			// TODO - none of these are ever changed
			bool preempted = false;
			bool timed_out = false;
			bool succeeded = false;
			//in loop, send PID enable commands to rotation, x, y
			while (ros::ok() && !preempted && !timed_out && !succeeded)
			{
				ROS_INFO_STREAM("----------------------------------------------");
				ROS_INFO_STREAM("current_position = " << odom_.pose.pose.position.x
								<< " " << odom_.pose.pose.position.y);

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
				// NOTE - there's code for this in base_trajectory.cpp - see
				// pointToLineSegmentDistance()
				const nav_msgs::Path path = spline_gen_srv.response.path;
				double minimum_distance = std::numeric_limits<double>::max();
				size_t minimum_idx = 0;

				const geometry_msgs::Point &odom_position = odom_.pose.pose.position;
				for (size_t i = 0; i < path.poses.size(); i++)
				{
					const geometry_msgs::Point &path_position = path.poses[i].pose.position;
					const double l2_dist = hypot(path_position.x - odom_position.x, path_position.y - odom_position.y);

					ROS_INFO_STREAM("waypoint " << i << " = " << path_position.x << ", " << path_position.y);
					ROS_INFO_STREAM("distance from waypoint " << i << " = " << l2_dist);
					if (l2_dist < minimum_distance)
					{
						minimum_distance = l2_dist;
						minimum_idx = i;
					}
				}
				ROS_INFO_STREAM("minimum_distance = " << minimum_distance);

				// Probably should also have a % of the distance traveled
				// between the two waypoints (or maybe just absolute distance)
				// - this will give an accurate
				// guess of where along the path we are (even if the robot
				// is off to one side or the other)
				const geometry_msgs::Pose &next_waypoint = path.poses[std::min(path.poses.size() - 1, minimum_idx + 1)].pose;

				// TODO - think about what the target point and axis are
				// We want to end up driving to a point on the path some
				// distance ahead of where we currently are
				// Since the segments are each straight lines, should be fairly
				// simple to advance some distance to find the target waypoint
				// Need to worry about coordinate frames, since the robot will
				// potentially be rotated such that it's x&y don't correspond
				// to the path x&y coordinate axes
				std_msgs::Bool enable_msg;
				enable_msg.data = true;
				std_msgs::Float64 command_msg;
				std_msgs::Float64 state_msg;

				auto x_axis_it = axis_states_.find("x");
				auto &x_axis = x_axis_it->second;
				x_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.x;
				x_axis.command_pub_.publish(command_msg);
				state_msg.data = odom_position.x;
				x_axis.state_pub_.publish(state_msg);

				auto y_axis_it = axis_states_.find("y");
				auto &y_axis = y_axis_it->second;
				y_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.y;
				y_axis.command_pub_.publish(command_msg);
				state_msg.data = odom_position.y;
				y_axis.state_pub_.publish(state_msg);

				auto z_axis_it = axis_states_.find("z");
				auto &z_axis = z_axis_it->second;
				z_axis.enable_pub_.publish(enable_msg);

				// TODO - what's the deal with yaw vs actual_yaw? And roll?
				double roll, pitch, yaw, current_yaw, target_yaw;
				tf::Quaternion odom_q(
					odom.pose.pose.orientation.w,
					odom.pose.pose.orientation.x,
					odom.pose.pose.orientation.y,
					odom.pose.pose.orientation.z);
				tf::Matrix3x3(odom_q).getRPY(target_yaw, pitch, yaw);
				tf::Quaternion waypoint_q(
					next_waypoint.orientation.w,
					next_waypoint.orientation.x,
					next_waypoint.orientation.y,
					next_waypoint.orientation.z);
				tf::Matrix3x3(waypoint_q).getRPY(target_yaw, pitch, yaw);

				command_msg.data = target_yaw;
				z_axis.command_pub_.publish(command_msg);
				state_msg.data = actual_yaw;
				z_axis.state_pub_.publish(state_msg);

				ros::spinOnce();
				r.sleep();

				// TODO - exit condition? Timeout? Check for prepemted?
				/*
				   if(minimum_idx == num_waypoints_ - 1)

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

				// Set the angle of the velocity
				geometry_msgs::Point32 base_link_waypoint;
				base_link_waypoint.x = next_waypoint.pose.position.x - odom.pose.pose.position.x;
				base_link_waypoint.y = next_waypoint.pose.position.y - odom.pose.pose.position.y;
				double mag = hypot(base_link_waypoint.x, base_link_waypoint.y);
				ROS_INFO_STREAM("distance to drive to next waypoint = " << mag);
				ROS_INFO_STREAM("max_velocity = " << max_velocity_);
				ROS_INFO_STREAM("distance to drive x = " << base_link_waypoint.x << "; distance to drive y = " << base_link_waypoint.y);
				                */
			}
			// TODO - disable all axes
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_server");

	ros::NodeHandle nh;
	PathAction path_action_server("path_server", nh);

	AlignActionAxisConfig x_axis("x", "x_enable_pub", "x_cmd_pub", "x_state_pub", "pid_debug", "x_timeout_param", "x_error_threshold_param");
	AlignActionAxisConfig y_axis("y", "y_enable_pub", "y_cmd_pub", "y_state_pub", "pid_debug", "y_timeout_param", "y_error_threshold_param");
	AlignActionAxisConfig z_axis("z", "z_enable_pub", "z_cmd_pub", "z_state_pub", "pid_debug", "z_timeout_param", "z_error_threshold_param");

	if (!path_action_server.addAxis(x_axis))
	{
		ROS_ERROR_STREAM("Error adding x_axis to path_action_server.");
		return false;
	}
	if (!path_action_server.addAxis(y_axis))
	{
		ROS_ERROR_STREAM("Error adding y_axis to path_action_server.");
		return false;
	}
	if (!path_action_server.addAxis(z_axis))
	{
		ROS_ERROR_STREAM("Error adding z_axis to path_action_server.");
		return false;
	}

	ros::spin();

	return 0;
}
