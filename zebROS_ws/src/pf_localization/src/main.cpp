/*
just a main function to test if code builds
also used to run some tests
*/

#include "particle_filter.hpp"
#include "world_model.hpp"
#include "particle.hpp"
#include <ros/ros.h>

#define VERBOSE

const std::string rot_topic = "zeroed_imu";
const std::string goal_pos_topic = "goal_detect_msg";
const std::string predicted_pos_topic = "pf/predicted_pos";

//formats and prints particle attributes
void print_particle(Particle p) {
  std::cout << p.x << ", " << p.y << ", " << p.rot << ", " << p.weight << '\n';
}

//print the attributes of all particles
void print_all_particles(ParticleFilter pf) {
  for (Particle p : pf.get_particles()) {
    print_particle(p);
  }
  std::cout << '\n';
}

int main(void) {
  std::mt19937 rng(0);

  

  std::vector<std::pair<double, double> > beacons;

  WorldModel world(beacons, 0, 16, 0, 16);
  ParticleFilter pf(world,
                    0, 16, 0, 16,
                    0.1, 0.1, 0.0,
                    200);

  std::pair<double, double> pos = std::make_pair(0, 0);

  while (ros::ok()) {
    std::vector<std::pair<double, double> > measurement;
    Particle p(pos.first, pos.second, 0.5);
    measurement = world.particle_relative(p);
    pf.assign_weights(measurement);
    pf.predict()
    pf.resample();
    pf.motion_update(pos.first - last_pos.first, pos.second - last_pos.second, 0);
    pf.set_rotation(0.5);
    std::cout << pos.first << ", " << pos.second << ", ";
  }
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
