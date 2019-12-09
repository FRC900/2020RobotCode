#include <ros/ros.h>
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

		ros::ServiceServer point_get_cli_;
		ros::ServiceClient spline_gen_cli_;

		ros::Subscriber odom_sub_;

	public:
		PathAction(const std::string &name) :
			as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start();

		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		point_gen_cli_ = nh_.serviceClient<swerve_point_generator::FullGenCoefs>("/path_to_goal/point_gen/command", false, service_connection_header);
		spline_gen_cli_ = nh_.serviceClient<base_trajectory::GenerateSpline>("/path_to_goal/base_trajectory/spline_gen", false, service_connection_header);

		odom_sub_ = nh_.subscribe("some odometry topic", 1, &PathAction::odomCallback, this);

                std::map<std::string, AlignActionAxisState> axis_states_;
	}

		~PathAction(void) {}

		void odomCallback(nav_msgs::Odometry odom_msg)
		{
			odom = odom_msg.pose.pose;
		}

                bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			double timeout;
			if(!nh_.getParam(axis_config.timeout_param_, timeout))
			{
				ROS_ERROR_STREAM("Could not read param "
						<< axis_config.timeout_param_
						<< " in align_server");
				//return false;
			}
			double error_threshold;
			if(!nh_.getParam(axis_config.error_threshold_param_, error_threshold))
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
						axis_config.error_sub_topic_,
						boost::bind(&BaseAlignAction::error_term_cb, this, _1, axis_config.name_),
						timeout,
						error_threshold)));
			return true;
		}

		void executeCB(const pure_pursuit::PathGoalConstPtr &goal)
		{
			// First, make the spline that represents the points we should travel to
			base_point_gen_srvectory::GenerateSpline spline_gen_srv;
			int point_num = goal->points.size();
			spline_gen_srv.request.points.resize(point_num);
			ROS_INFO_STREAM(req.points[0].x << " " << req.points[0].y << " " << req.points[0].z);

			for(int i = 0; i<point_num; i++)
			{
				//x-movement
				spline_gen_srv.request.points[i].positions.push_back(req.points[i].x);
				//y-movement
				spline_gen_srv.request.points[i].positions.push_back(req.points[i].y);
				//z-rotation
				spline_gen_srv.request.points[i].positions.push_back(req.points[i].z);
				//time for profile to run
				spline_gen_srv.request.points[i].time_from_start = ros::Duration(10*(i+1)); //TODO this is totally arbitrary
			}

			if(!spline_gen.call(spline_gen_srv))
			{
				ROS_ERROR_STREAM("Call to spline_gen failed in path action server");
			}

			// Second, find the points from the spline
			swerve_point_generator::FullGenCoefs point_gen_srv;

			point_gen_srv.request.orient_coefs.resize(point_num);
			point_gen_srv.request.x_coefs.resize(point_num);
			point_gen_srv.request.y_coefs.resize(point_num);
			point_gen_srv.request.end_points.resize(point_num);
			point_gen_srv.request.spline_groups.push_back(point_num);

			ROS_INFO_STREAM("srv base point_gen_srvectory size = " << spline_gen_srv.response.orient_coefs.size());

			for(int p = 0; p < point_num; p++)
			{
				for(size_t i = 0; i < spline_gen_srv.response.orient_coefs[p].spline.size(); i++)
				{
					ROS_INFO_STREAM("p = " << p << " i = " << i);
					point_gen_srv.request.orient_coefs[p].spline.push_back(spline_gen_srv.response.orient_coefs[p].spline[i]);
					ROS_INFO_STREAM("p = " << p << " i = " << i);
					point_gen_srv.request.x_coefs[p].spline.push_back(spline_gen_srv.response.x_coefs[p].spline[i]);
					ROS_INFO_STREAM("p = " << p << " i = " << i);
					point_gen_srv.request.y_coefs[p].spline.push_back(spline_gen_srv.response.y_coefs[p].spline[i]);
					ROS_INFO_STREAM("p = " << p << " i = " << i);
				}

				point_gen_srv.request.wait_before_group.push_back(p*0.16);
				point_gen_srv.request.t_shift.push_back(0);
				point_gen_srv.request.flip.push_back(false);
				point_gen_srv.request.end_points[p] = spline_gen_srv.response.end_points[p+1];
				point_gen_srv.request.initial_v = 0;
				point_gen_srv.request.final_v = 0;
				point_gen_srv.request.x_invert.push_back(0);
			}

			if(!point_gen_cli_.call(traj))
			{
				ROS_ERROR_STREAM("Failed to call point gen service in path action");
			}

			double dt = traj.response.dt;
			trajectory_msgs::JointTrajectoryPoint[] points = traj.response.points;

			//in loop, send PID enable commands to rotation, x, y
			while(ros::ok() && !preempted_ && !timed_out_ && !succeeded_)
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

                                std_msgs::Bool enable_msg;
                                enable_msg.data = true;
                                std_msgs::Float64 command_msg;

                                auto axis_it = axis_states_.find("x");
                                auto &axis = axis_it->second;
                                axis.enable_pub_.publish(enable_msg); 
                                command_msg.data = next_waypoint.Pose.Point.x;
                                axis.command_pub_.publish(command_msg);

                                auto axis_it = axis_states_.find("y");
                                auto &axis = axis_it->second;
                                axis.enable_pub_.publish(enable_msg); 
                                command_msg.data = next_waypoint.Pose.Point.y;
                                axis.command_pub_.publish(command_msg);

                                auto axis_it = axis_states_.find("z");
                                auto &axis = axis_it->second;
                                axis.enable_pub_.publish(enable_msg); 

                                double roll, pitch, yaw, actual_yaw;
                                tf::Quaternion waypoint_q(
                                        next_waypoint.pose.orientation.w,
                                        next_waypoint.pose.orientation.x,
                                        next_waypoint.pose.orientation.y,
                                        next_waypoint.pose.orientation.z);
                                tf::Matrix3x3(odom_q).getRPY(actual_yaw, pitch, yaw);

                                command_msg.data = actual_yaw; 
                                axis.command_pub_.publish(command_msg);

                                r.sleep();
                                ros::spinOnce();
                                
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
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");

	PathAction path_action_server("path_server");

	ros::NodeHandle nh;

        AlignActionAxisConfig x_axis("x", "x_enable_pub", "x_error_sub", "x_timeout_param", "x_error_threshold_param");
        AlignActionAxisConfig y_axis("y", "y_enable_pub", "y_error_sub", "y_timeout_param", "y_error_threshold_param");
        AlignActionAxisConfig z_axis("z", "z_enable_pub", "z_error_sub", "z_timeout_param", "z_error_threshold_param");

        if(!path_action_server.addAxis(x_axis))
        {
            ROS_ERROR_STREAM("Error adding x_axis to path_action_server.");
            return false;
        }
        if(!path_action_server.addAxis(y_axis))
        {
            ROS_ERROR_STREAM("Error adding y_axis to path_action_server.");
            return false;
        }
        if(!path_action_server.addAxis(z_axis))
        {
            ROS_ERROR_STREAM("Error adding z_axis to path_action_server.");
            return false;
        }

        ros::spin();

        return 0;
}
