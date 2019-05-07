/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 * 4. transform goal point to vehicle coordinates and drive to that position
 */
#include "pure_pursuit/pure_pursuit.h"

void PurePursuit::setup()
{
    // Read config values
    nh_.getParam("/pure_pursuit/lookahead_distance", lookahead_distance_);
    nh_.getParam("/swerve_drive_controller/max_speed", max_velocity_);
    nh_.getParam("/swerve_drive_controller/max_accel", max_accel_);
    nh_.getParam("/pure_pursuit/pos_tol", pos_tol_);

}

// TODO : for large function parameters, making them const T & is more efficient
void PurePursuit::loadPath(nav_msgs::Path path)
{
    path_ = path;
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a Twist (or just x,y
// velocity) and whoever called run would be responsible for sending that
// where it needs to go.
geometry_msgs::Twist PurePursuit::run(nav_msgs::Odometry odom)
{
	  geometry_msgs::PoseStamped next_waypoint;
      ros::spinOnce();

      // Find point in path closest to odometry reading
      double minimum_distance = std::numeric_limits<double>::max();
      size_t minimum_idx = std::numeric_limits<size_t>::max();
      for(int i = 0; i < path_.poses.size(); i++)
      {
          if(hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y) < minimum_distance)
          {
              minimum_distance = hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y - odom.pose.pose.position.y);
              minimum_idx = i;
          }
      }
      
      // Find the closest point on the path within the lookahead distance and use that as our next waypoint
      if(minimum_distance > pos_tol_)
      {
          for(int i = 0; i < path_.poses.size(); i++)
          {
              if(hypot(path_.poses[i].pose.position.x - odom.pose.pose.position.x, path_.poses[i].pose.position.y- odom.pose.pose.position.y) > lookahead_distance_)
              {
                  next_waypoint = path_.poses[i - 1];
                  break;
              }
          }
      }
      else // if we are close enough
      {
          next_waypoint = path_.poses[minimum_idx + 1];
      }

      // Set the angle of the velocity
      geometry_msgs::Point32 base_link_waypoint;
      base_link_waypoint.x = next_waypoint.pose.position.x - odom.pose.pose.position.x;
      base_link_waypoint.y = next_waypoint.pose.position.y - odom.pose.pose.position.y;
      double mag = hypot(base_link_waypoint.x, base_link_waypoint.y);
      cmd_vel_.linear.x = max_velocity_ * base_link_waypoint.x / mag;
      cmd_vel_.linear.y = max_velocity_ * base_link_waypoint.y / mag;
      cmd_vel_.linear.z = 0;
      cmd_vel_.angular.x = 0;
      cmd_vel_.angular.y = 0;
      cmd_vel_.angular.z = 0;

	  return cmd_vel_;
}
