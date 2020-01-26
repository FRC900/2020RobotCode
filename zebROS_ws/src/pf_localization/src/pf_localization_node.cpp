#include "particle_filter.hpp"
#include "world_model.hpp"
#include "particle.hpp"
#include "pf_localization/pf_pose.h"
#include "goal_detection/GoalDetection.h"
#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <utility>

#define VERBOSE

const std::string rot_topic = "zeroed_imu";
const std::string odom_topic = "odom";
const std::string goal_pos_topic = "goal_detect_msg";

const std::string pub_topic = "pf/predicted_pose";
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

=======
void rotCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double roll, pitch, yaw;
  tf2::Quaternion raw;
  tf2::convert(msg -> orientation, raw);
  tf2::Matrix3x3(raw).getRPY(roll, pitch, yaw);
  rot = degToRad(yaw);
}

void goalCallback(const goal_detection::GoalDetection::ConstPtr& msg){
  for(const geometry_msgs::Point32& p : msg->location) {
    measurement.push_back(std::make_pair(p.x, p.y));
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  delta_x = msg->pose.pose.position.x;
  delta_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_localization_node");
  ros::NodeHandle nh_;
  ros::Subscriber rot_sub = nh_.subscribe(rot_topic, 1000, rotCallback);
  ros::Subscriber odom_sub = nh_.subscribe(odom_topic, 1000, odomCallback);
  ros::Subscriber goal_sub = nh_.subscribe(goal_pos_topic, 1000, goalCallback);

  pub_ = nh_.advertise<pf_localization::pf_pose>(pub_topic, 1);

  std::vector<std::pair<double, double> > beacons;
  for (int i = 0; i < 10; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16,
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16
    ));
  }
  WorldModel world(beacons, 0, 16, 0, 16);
  ParticleFilter pf(world,
                    0, 8, 2, 4,
                    0.1, 0.1, 0.1,
                    200);

  #ifdef VERBOSE
  for (Particle p : pf.get_particles()) {
    print_particle(p);
  }
  ROS_INFO_STREAM("\n\n");
  print_particle(pf.predict());
  #endif


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
  }

  return 0;
}
