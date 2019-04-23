/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 * 4. transform goal point to vehicle coordinates and drive to that position
 */

/* Pseudocode 1
 * while(path is achievable, timeout is not reached, nothing is preempted, path isn't finished)
 *      // STEP TWO
 *      currentPos, pathPoints
 *      for point in pathPoints:
 *          if dist(current_pos, point) < min,
 *              index of closest point  = i
 *              min_dist = dist(current_pos, min)
 *
 *      if(min_dist < we_don't_care_value):
 *          next_waypoint = pathPoints[i+]
 *
 *      //STEP 2.5 (change the lookahead velocity)
 *      if(linear_speed < min):
 *          desired_lookahead = lookahead_min
 *      else if (linear_speed > max):
 *          desired_lookahead = lookahead_max
 *      else:
 *          desired_lookahead = speed
 *
 *      if(desired_lookahead > actual_lookahead)
 *          actual_lookahead += 0.01
 *      else
 *          actual_lookahead -= 0.01
 *      
 *
 *      if(min_dist > we_don't_care_value):
 *      // STEP THREE
 *          for point in pathPoints[i:]:
 *              if dist(pathPoints[i], point) < lookahead_distance && > max
 *                  next_waypoint = pathPoints[j]
 *
 *      //STEP FOUR -- send the next position / velocity waypoint to the swerve drive controller to execute
 *      maxSpeed
 *      goto_point = next_waypoint - current_position
 *      rhat = goto_point / mag(goto_point)
 *      velocity_vector = rhat * maxSpeed
 *      
 *      publishTwistMessage(veloctiy_vector)
 *
 *      if(next_waypoint) is the last waypoint:
 *          break;
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <string>

class PurePursuit
{
public:
  PurePursuit(ros::NodeHandle nh, nav_msgs::Path path);

  // Runs whenever an odometry message is received, contains the main control loop
  void callback(nav_msgs::Odometry odom);
  
  // Helper founction for computing euclidian distances in the x-y plane.
  double dist(geometry_msgs::Point32 pt1, geometry_msgs::Point32 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2));
  }

  // Run the controller
  void run()
  {
      ros::spin();
  }
  
private:
  nav_msgs::Path path_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  
  // ROS infrastructure
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_vel_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string map_frame_id_, robot_frame_id_;
  double lookahead_distance_;
  std::string odometry_topic_;
  double max_velocity_, max_accel_;
  double pos_tol_;
};

PurePursuit::PurePursuit(ros::NodeHandle nh, nav_msgs::Path path)
{
    // Read config values
    nh.getParam("lookahead_distance", lookahead_distance_);
    nh.getParam("odometry_topic", odometry_topic_);
    nh.getParam("max_velocity", max_velocity_);
    nh.getParam("max_accel", max_accel_);
    nh.getParam("robot_frame", robot_frame_id_);
    nh.getParam("map_frame", map_frame_id_);

    // Read in path
    path_ = path;

    // Set up publishers, subscribers
    odom_sub_.subcribe(odometry_topic, 1, callback);
    cmd_vel_pub_.advertise("swerve_drive_controller/cmd_vel", 1);
}

PurePursuit::callback(nav_msgs::Odometry odom)
{
      // Find point in path closest to odometry reading
      double minimum_distance = std::numeric_limits<double>::max();
      size_t minimum_idx = std::numeric_limits<size_t>::max();
      for(int i = 0; i < path_.poses.size(); i++)
      {
          if(dist(path.poses[i], odom.pose.position) < minimum_distance)
          {
              minimum_distance = dist(path.poses[i], odom.pose.position);
              minumum_idx = i;
          }
      }
      
      // Find the closest point on the path within the lookahead distance and use that as our next waypoint
      if(minimum_distance > pos_tol)
      {
          for(int i = 0; i < path_.poses.size(); i++)
          {
              if(dist(path_.poses[i], odom.position.pose) > lookahead_distance)
              {
                  next_waypoint = path_.poses[i - 1];
                  break;
              }
          }
      }
      else // if we are close enough
      {
          next_waypoint = path.poses[minimum_idx + 1];
      }

      // Set the angle of the velocity
      geometry_msgs::Point32 base_link_waypoint;
      tf.tranform();//something here to make the next_waypoint be in the frame of odometry
      mag = sqrt(pow(base_link_waypoint.x, 2) + pow(base_link_waypoint.y, 2));
      cmd_vel_.linear.x = max_speed * base_link_waypoint.x / mag;
      cmd_vel_.linear.y = max_speed * base_link_waypoint.y / mag;
      cmd_vel_.linear.z = 0;
      cmd_vel_.angular.x = 0;
      cmd_vel_.angular.y = 0;
      cmd_vel_.angular.z = 0;

      cmd_vel_pub_.publish(cmd_vel);
}
