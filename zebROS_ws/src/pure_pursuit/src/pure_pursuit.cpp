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
//tf stuff
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class PurePursuit
{
public:
  PurePursuit(ros::NodeHandle nh)
  {
      nh_ = nh;
  }

  void setup();

  void loadPath(nav_msgs::Path path);

  // Runs whenever an odometry message is received
  void odomCallback(nav_msgs::OdometryConstPtr odom);
  
  // Helper founction for computing euclidian distances in the x-y plane.
  // TODO : use hypot() function
  template<typename T1, typename T2>
  double dist(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2));
  }

  // contains the main control loop
  void run();
  
private:
  nav_msgs::Path path_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  
  // ROS infrastructure
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  tf2_ros::Buffer buffer_;
  std::string target_frame_;
  double lookahead_distance_;
  std::string odometry_topic_;
  double max_velocity_, max_accel_;
  double pos_tol_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::PoseStamped next_waypoint_; // TODO : make a local var

  std::shared_ptr<tf2_ros::TransformListener> tf2_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::Odometry>> tf2_filter_;
};

void PurePursuit::setup()
{
    // Read config values
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("odometry_topic", odometry_topic_);
    nh_.getParam("max_velocity", max_velocity_);
    nh_.getParam("max_accel", max_accel_);
    nh_.getParam("target_frame", target_frame_); //position of the robot at the beginning of the path
    nh_.getParam("pos_tol", pos_tol_);

    // Set up publishers, subscribers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

    // Subscribing to odometry/odom transforms
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "pointstamped_goal_msg", 1);
    tf2_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<nav_msgs::Odometry>>(buffer_, target_frame_, 10, nh_);
    tf2_filter_->registerCallback(boost::bind(&PurePursuit::odomCallback, this, _1));
}

// TODO : for large function parameters, making them const T & is more efficient
void PurePursuit::loadPath(nav_msgs::Path path)
{
    path_ = path;
}

void PurePursuit::odomCallback(nav_msgs::OdometryConstPtr odom)
{
    try
    {
        buffer_.transform(*odom, odom_msg_, target_frame_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Failed %s\n", ex.what());
    }
}

// TODO : try to decouple PurePursuit from specific topics
// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a Twist (or just x,y
// velocity) and whoever called run would be responsible for sending that
// where it needs to go.
//
void PurePursuit::run()
{
      ros::spinOnce();

      // Find point in path closest to odometry reading
      double minimum_distance = std::numeric_limits<double>::max();
      size_t minimum_idx = std::numeric_limits<size_t>::max();
      for(int i = 0; i < path_.poses.size(); i++)
      {
          if(dist(path_.poses[i].pose.position, odom_msg_.pose.pose.position) < minimum_distance)
          {
              minimum_distance = dist(path_.poses[i].pose.position, odom_msg_.pose.pose.position);
              minimum_idx = i;
          }
      }
      
      // Find the closest point on the path within the lookahead distance and use that as our next waypoint
      if(minimum_distance > pos_tol_)
      {
          for(int i = 0; i < path_.poses.size(); i++)
          {
              if(dist(path_.poses[i].pose.position, odom_msg_.pose.pose.position) > lookahead_distance_)
              {
                  next_waypoint_ = path_.poses[i - 1];
                  break;
              }
          }
      }
      else // if we are close enough
      {
          next_waypoint_ = path_.poses[minimum_idx + 1];
      }

      // Set the angle of the velocity
      geometry_msgs::Point32 base_link_waypoint;
      //tf.tranform();//something here to make the next_waypoint be in the frame of odometry
      base_link_waypoint.x = next_waypoint_.pose.position.x - odom_msg_.pose.pose.position.x;
      base_link_waypoint.y = next_waypoint_.pose.position.y - odom_msg_.pose.pose.position.y;
      double mag = sqrt(pow(base_link_waypoint.x, 2) + pow(base_link_waypoint.y, 2)); // TODO : use hypot()
      cmd_vel_.linear.x = max_velocity_ * base_link_waypoint.x / mag;
      cmd_vel_.linear.y = max_velocity_ * base_link_waypoint.y / mag;
      cmd_vel_.linear.z = 0;
      cmd_vel_.angular.x = 0;
      cmd_vel_.angular.y = 0;
      cmd_vel_.angular.z = 0;

      cmd_vel_pub_.publish(cmd_vel_);
}
