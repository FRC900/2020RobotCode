/*
just a main function to test if code builds
also used to run some tests
*/

#include "particle_filter.hpp"
#include "world_model.hpp"
#include <vector>
#include <utility>
#include <iostream>

void print_particle(Particle p) {
  std::cout << p.x << ", " << p.y << ", " << p.rot << ", " << p.weight << '\n';
}

void print_all_particles(ParticleFilter pf) {
  for (Particle p : pf.get_particles()) {
    print_particle(p);
  }
  std::cout << '\n';
}

int main(int argc, char const *argv[]) {
  #if 0
  // test if builds
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world, 0, 0, 0.5, 0.5, 0.1, 0.1, 0.1, 100);
  #endif

  #if 0
  // test motion_update
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world,
                    0, 0.5, 0, 0.5,
                    0.01, 0.01, 0.01,
                    1);
  print_all_particles(pf);
  for (int i = 0; i < 10; i++) {
    pf.motion_update(0, 0.01, 0);
    print_all_particles(pf);
  }
  #endif

  #if 0
  // test set_rotation
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world,
                    0, 0.5, 0, 0.5,
                    0.01, 0.01, 0.01,
                    1);
  print_all_particles(pf);
  pf.set_rotation(1);
  print_all_particles(pf);
  #endif

  #if 1
  // test localization around fixed point
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world,
                    0, 0.5, 0, 0.5,
                    0.01, 0.01, 0.01,
                    5);
  print_all_particles(pf);
  /*
  for (int i = 0; i < 10; i++) {
    std::vector<std::pair<double, double> > measurement;
    measurement.push_back(std::make_pair(-0.5, -0.5));
    pf.assign_weights(measurement);
    pf.resample();
    pf.motion_update(0, 0, 0);
    print_all_particles(pf);
  }
  */

  #endif

  return 0;
}
