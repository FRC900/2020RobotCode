#include "particle_filter.hpp"
#include "world_model.hpp"
#include "particle.hpp"
#include "pf_localization/pf_pose.h"

#include <iostream>
#include <ros/ros.h>

#define VERBOSE

const std::string rot_topic = "zeroed_imu";
const std::string odom_topic = "odom";
const std::string goal_pos_topic = "goal_detect_msg";
const std::string pub_topic = "pf/predicted_pose";
static ros::Publisher pub;

//formats and prints particle attributes
void print_particle(const Particle& p) {
  std::cout << p.x << ", " << p.y << ", " << p.rot << ", " << p.weight << '\n';
}

int main(int argc, char const *argv[]) {
  #if 0
  std::mt19937 rng(0);
=======
int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_localization_node");
  ros::NodeHandle nh_;
>>>>>>> updated cmakelist and package.xml for pf localization.. it builds!:zebROS_ws/src/pf_localization/src/pf_localization_node.cpp

  pub = nh_.advertise<pf_localization::pf_pose>(pub_topic, 1);

  std::mt19937 rng(0);
  std::vector<std::pair<double, double> > beacons;
  for (int i = 0; i < 10; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16,
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16
    ));
  }
  WorldModel world(beacons, 0, 16, 0, 16);
=======
  WorldModel world(beacons, 0, 8.2, 0, 16.0);
>>>>>>> updated cmakelist and package.xml for pf localization.. it builds!:zebROS_ws/src/pf_localization/src/pf_localization_node.cpp
  ParticleFilter pf(world,
                    0, 8, 2, 4,
                    0.1, 0.1, 0.0,
                    200);

  std::pair<double, double> pos = std::make_pair(0, 0);

  while (ros::ok()) {
    std::vector<std::pair<double, double> > measurement;
    Particle p(pos.first, pos.second, 0.5);
    measurement = world.particle_relative(p);
    pf.assign_weights(measurement);
    pf.predict();
    pf.resample();
    //pf.motion_update(pos.first - last_pos.first, pos.second - last_pos.second, 0);
    pf.set_rotation(0.5);
    std::cout << pos.first << ", " << pos.second << ", ";
  }
  #endif

  #if 0
  // test circular movement with output for animated visualization
  // NOTE: this sends a LOT of stuff to stdout, redirect it to a file

  std::mt19937 rng(0);
  std::vector<std::pair<double, double> > beacons;
  for (int i = 0; i < 10; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16,
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16
    ));
  }
  WorldModel world(beacons, 0, 16, 0, 16);
  ParticleFilter pf(world,
                    0, 16, 0, 16,
                    0.1, 0.1, 0.1,
                    200);

  std::vector<double> boundaries = world.get_boundaries();
  std::cout << boundaries[0] << ',' << boundaries[1] << ',' << boundaries[2] << ',' << boundaries[3] << "\n*\n";
  for (std::pair<double, double> b : beacons) {
    std::cout << b.first << ',' << b.second << '\n';
  }
  std::cout << "\n*\n";
  double theta = 0;
  const double r = 3;
  double theta_step = 0.01;
  const int iterations = 628;
  std::pair<double, double> pos = std::make_pair(11, 8);
  std::pair<double, double> last_pos;
  for (int i = 0; i < iterations; i++) {
    last_pos = pos;
    pos = std::make_pair(r * cos(theta) + 8, r * sin(theta) + 8);
    std::vector<std::pair<double, double> > measurement;
    Particle p(pos.first, pos.second, theta + 3.14159265258979323 / 2);
    measurement = world.particle_relative(p);
    pf.assign_weights(measurement);
    Particle prediction = pf.predict();
    pf.resample();
    pf.motion_update(hypot(pos.first - last_pos.first, pos.second - last_pos.second), 0, theta_step);
    print_all_particles(pf);
    print_particle(pf.predict());
    std::cout << pos.first << ", " << pos.second << ", " << theta + 3.14159265258979323 / 2 << "\n~\n";
    print_particle(prediction);
    //std::cout << "\n";
    theta += theta_step;
  }
  #endif

  #if 0
  // test motion update with output for animated visualization
  // NOTE: this sends a LOT of stuff to stdout, redirect it to a file

  std::mt19937 rng(0);
  std::vector<std::pair<double, double> > beacons;
  for (int i = 0; i < 10; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16,
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16
    ));
  }
  WorldModel world(beacons, 0, 16, 0, 16);
  ParticleFilter pf(world,
                    10.7, 10.9, 7.9, 8.1,
                    0.0, 0.0, 0.0,
                    200);

  std::vector<double> boundaries = world.get_boundaries();
  std::cout << boundaries[0] << ',' << boundaries[1] << ',' << boundaries[2] << ',' << boundaries[3] << "\n*\n";
  for (std::pair<double, double> b : beacons) {
    std::cout << b.first << ',' << b.second << '\n';
  }
  std::cout << "\n*\n";
  double theta = 0;
  const double r = 3;
  double theta_step = 0.01;
  const int iterations = 628;
  std::pair<double, double> pos = std::make_pair(11.0, 8.0);
  std::pair<double, double> last_pos;
  for (int i = 0; i < iterations; i++) {
    last_pos = pos;
    pos = std::make_pair(r * cos(theta) + 8, r * sin(theta) + 8);
    if (i == 0) {
      pf.set_rotation(3.14159265258979323 / 2);
      std::vector<std::pair<double, double> > measurement;
      Particle p(pos.first, pos.second, 3.14159265258979323 / 2);
      measurement = world.particle_relative(p);
      pf.assign_weights(measurement);
    }
    Particle prediction = pf.predict();
    pf.motion_update(hypot(pos.first - last_pos.first, pos.second - last_pos.second), 0, theta_step);
    print_all_particles(pf);
    print_particle(pf.predict());
    // pf.set_rotation(0.5);
    std::cout << pos.first << ", " << pos.second << ", " << theta + 3.14159265258979323 / 2 << "\n~\n";
    print_particle(prediction);
    //std::cout << "\n";
    theta += theta_step;
  }
  #endif

  #if 1
  // test localization using actual retro tape beacon positions
  // NOTE: this sends a LOT of stuff to stdout, redirect it to a file

  std::mt19937 rng(0);
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(2.40, 0));
  beacons.push_back(std::make_pair(6.50, 0));
  beacons.push_back(std::make_pair(1.71, 15.98));
  beacons.push_back(std::make_pair(5.81, 15.98));
  WorldModel world(beacons, 0, 8.21, 0, 15.98);
  ParticleFilter pf(world,
                    0, 8.21, 2, 4,
                    0.1, 0.1, 0.1,
                    200);

  std::vector<double> boundaries = world.get_boundaries();
  std::cout << boundaries[0] << ',' << boundaries[1] << ',' << boundaries[2] << ',' << boundaries[3] << "\n*\n";
  for (std::pair<double, double> b : beacons) {
    std::cout << b.first << ',' << b.second << '\n';
  }
  std::cout << "\n*\n";
  const double r = 3;
  double theta_step = 0.01;
  const int iterations = 628;
  std::pair<double, double> pos = std::make_pair(4, 3.05);
  double theta = -3.14159265258979323 / 2;
  std::pair<double, double> last_pos;
  for (int i = 0; i < iterations; i++) {
    last_pos = pos;
    pos = std::make_pair(r * cos(theta) + 4, r * sin(theta) + 6.05);
    std::vector<std::pair<double, double> > measurement;
    Particle p(pos.first, pos.second, theta + 3.14159265258979323 / 2);
    measurement = world.particle_relative(p);
    pf.assign_weights(measurement);
    Particle prediction = pf.predict();
    pf.resample();
    pf.motion_update(hypot(pos.first - last_pos.first, pos.second - last_pos.second), 0, theta_step);
    print_all_particles(pf);
    print_particle(pf.predict());
    std::cout << pos.first << ", " << pos.second << ", " << theta + 3.14159265258979323 / 2 << "\n~\n";
    print_particle(prediction);
    //std::cout << "\n";
    theta += theta_step;
  }
  #endif

  return 0;
}

/*
#if 1
std::mt19937 rng(0);
// test circular movement
std::vector<std::pair<double, double> > beacons;
for (int i = 0; i < 10; i++) {
  beacons.push_back(std::make_pair(
    ((double) rng() - rng.min()) / (rng.max() - rng.min()),
    ((double) rng() - rng.min()) / (rng.max() - rng.min())
  ));
}
WorldModel world(beacons, 0, 16, 0, 16);
ParticleFilter pf(world,
                  0, 16, 0, 16,
                  0.1, 0.1, 0.0,
                  200);

double theta = 0;
const double r = 3;
double theta_step = 0.01;
std::pair<double, double> pos = std::make_pair(13, 8);
std::pair<double, double> last_pos;
for (int i = 0; i < 628; i++) {
  last_pos = pos;
  pos = std::make_pair(r * cos(theta) + 8, r * sin(theta) + 8);
  std::vector<std::pair<double, double> > measurement;
  Particle p(pos.first, pos.second, 0.5);
  measurement = world.particle_relative(p);
  // for (std::pair<double, double> b : beacons) {
  //   measurement.push_back(std::make_pair(b.first - pos.first, b.second - pos.second));
  // }
  pf.assign_weights(measurement);
  pf.resample();
  pf.motion_update(pos.first - last_pos.first, pos.second - last_pos.second, 0);
  pf.set_rotation(0.5);
  std::cout << pos.first << ", " << pos.second << ", ";
  print_particle(pf.predict());
  //std::cout << "\n";
  theta += theta_step;
}
#endif
*/
