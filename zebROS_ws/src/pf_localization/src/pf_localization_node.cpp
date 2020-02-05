#include "particle_filter.hpp"
#include "world_model.hpp"
#include "particle.hpp"
#include "pf_localization/pf_pose.h"
#include "goal_detection/GoalDetection.h"
#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <XmlRpcValue.h>

#define VERBOSE
#define EXTREME_VERBOSE


const std::string rot_topic = "/navx_jetson/zeroed_imu";
const std::string cmd_topic = "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out";
const std::string goal_pos_topic = "/goal_detection/goal_detect_msg";

const std::string pub_topic = "predicted_pose";
static ros::Publisher pub_;

constexpr double pi = 3.14159;

double delta_x = 0;
double delta_y = 0;
double rot = 0;
std::vector<std::pair<double, double> > measurement;

double degToRad(double deg) {
  double rad = (deg / 180) * pi;
  return rad;
}

//formats and prints particle attributes
void print_particle(Particle p) {
  ROS_INFO_STREAM(p.x << ", " << p.y << ", " << p.rot << ", " << p.weight << '\n');
}

void rotCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double roll, pitch, yaw;
  tf2::Quaternion raw;
  tf2::convert(msg -> orientation, raw);
  tf2::Matrix3x3(raw).getRPY(roll, pitch, yaw);
  rot = degToRad(yaw);
  #ifdef EXTREME_VERBOSE
  ROS_INFO("rotCallback called");
  #endif
}

void goalCallback(const goal_detection::GoalDetection::ConstPtr& msg){
  measurement.clear();
  for(const goal_detection::Goal& p : msg->goals) {
    measurement.push_back(std::make_pair(p.location.y, p.location.x));
  }
  #ifdef EXTREME_VERBOSE
  ROS_INFO("goalCallback called");
  #endif
}

void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  double timestep = msg->header.stamp.sec;
  double x_vel = msg->twist.linear.x;
  double y_vel = msg->twist.linear.y;

  delta_x += x_vel * timestep;
  delta_y += y_vel * timestep;

  #ifdef EXTREME_VERBOSE
  ROS_INFO("cmdCallback called");
  #endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_localization_node");
  ros::NodeHandle nh_;

  #ifdef VERBOSE
  ROS_INFO_STREAM(nh_.getNamespace());
  #endif

  XmlRpc::XmlRpcValue xml_beacons;
  double f_x_min, f_x_max, f_y_min, f_y_max, i_x_min, i_x_max, i_y_min, i_y_max, p_stdev, r_stdev;
  int num_particles;

  std::vector<std::pair<double, double> > beacons;

  nh_.getParam("num_particles", num_particles);
  nh_.getParam("beacons", xml_beacons);

  if (!nh_.getParam("field_dims/x_min", f_x_min)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/x_max", f_x_max)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/y_min", f_y_min)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/y_max", f_y_max)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

  ROS_INFO_STREAM("field dims assigned");

  if (!nh_.getParam("init_dims/x_min", i_x_min)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/x_max", i_x_max)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/y_min", i_y_min)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/y_max", i_y_max)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }

  ROS_INFO("initialization dims assigned");

  if (!nh_.param("noise_stdev/position", p_stdev, 0.1)) {
    ROS_WARN("no position stdev specified, using defalut");
  }
  if (!nh_.param("noise_stdev/rotation", r_stdev, 0.1)) {
    ROS_WARN("no rotation stdev specified, using defalut");
  }

  ROS_INFO("noise stdevs assigned");

  ROS_INFO_STREAM(f_x_min << ' ' << i_x_min << ' ' << p_stdev);

  beacons.reserve(xml_beacons.size());
  for (size_t i = 0; i < (unsigned) xml_beacons.size(); i++) {
    beacons.push_back(std::make_pair(xml_beacons[i][0], xml_beacons[i][1]));
  }

  WorldModel world(beacons, f_x_min, f_x_max, f_y_min, f_y_max);
  ParticleFilter pf(world,
                    i_x_min, i_x_max, i_y_min, i_y_max,
                    p_stdev, r_stdev,
                    num_particles);

  #ifdef VERBOSE
  for (Particle p : pf.get_particles()) {
    print_particle(p);
  }
  ROS_INFO_STREAM("\n\n");
  print_particle(pf.predict());
  ROS_INFO("pf localization initialized");
  #endif

  ros::Subscriber rot_sub = nh_.subscribe(rot_topic, 10, rotCallback);
  ros::Subscriber odom_sub = nh_.subscribe(cmd_topic, 10, cmdCallback);
  ros::Subscriber goal_sub = nh_.subscribe(goal_pos_topic, 10, goalCallback);

  pub_ = nh_.advertise<pf_localization::pf_pose>(pub_topic, 1);


  ros::Rate rate(10);
  while (ros::ok()) {
    pf.set_rotation(rot);
    pf.motion_update(delta_x, delta_y, 0);
    if (measurement.size() > 0){
      pf.assign_weights(measurement);
    }
    Particle prediction = pf.predict();
    if (measurement.size() > 0){
      pf.resample();
    }

    pf_localization::pf_pose pose;
    pose.x = prediction.x;
    pose.y = prediction.y;
    pose.rot = prediction.rot;
    pub_.publish(pose);

    delta_x = 0;
    delta_y = 0;
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
