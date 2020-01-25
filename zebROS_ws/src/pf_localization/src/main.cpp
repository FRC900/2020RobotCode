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
void print_particle(const Particle& p) {
  std::cout << p.x << ", " << p.y << ", " << p.rot << ", " << p.weight << '\n';
}

//print the attributes of all particles
void print_all_particles(const ParticleFilter& pf) {
  for (Particle p : pf.get_particles()) {
    print_particle(p);
  }
}

<<<<<<< HEAD
int main(void) {
=======
int main(int argc, char const *argv[]) {
  #if 0
>>>>>>> implemented robot-relative motion update
  std::mt19937 rng(0);

  

  std::vector<std::pair<double, double> > beacons;
<<<<<<< HEAD

=======
  for (int i = 0; i < 10; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16,
      ((double) rng() - rng.min()) / (rng.max() - rng.min()) * 16
    ));
  }
>>>>>>> implemented robot-relative motion update
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
    // pf.set_rotation(0.5);
    std::cout << pos.first << ", " << pos.second << ", ";
  }
<<<<<<< HEAD
=======
  #endif

  #if 1
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

>>>>>>> implemented robot-relative motion update
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
