#pragma once

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
  PurePursuit(ros::NodeHandle nh)
  {
      nh_ = nh;
  }

  void setup();

  void loadPath(nav_msgs::Path path);

  // contains the main control loop
  geometry_msgs::Twist run(nav_msgs::Odometry odom);
  
private:
  nav_msgs::Path path_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  
  // ROS infrastructure
  ros::NodeHandle nh_;
  double lookahead_distance_;
  double max_velocity_, max_accel_;
  double pos_tol_;
};

