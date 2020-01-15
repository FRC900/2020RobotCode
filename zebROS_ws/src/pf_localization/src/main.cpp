/*
just a main function to test if code builds
also used to run some tests
*/

#include "particle_filter.hpp"
#include "world_model.hpp"
#include "particle.hpp"
#include <vector>
#include <utility>
#include <iostream>
#include <random>

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

  #if 0
  // test if world model total distance works
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  // beacons.push_back(std::make_pair(3.0, 4.0));
  WorldModel world(beacons, 0, 10, 0, 10);
  Particle p0 { 0, 4, 0 };
  Particle p1 = p0;
  // Particle p2 { 0, 100, 0 };
  std::vector<std::pair<double, double> > m;
  m.push_back(std::make_pair(-3.0, 0.0));
  m.push_back(std::make_pair(0.0, 4.0));
  std::cout << world.total_distance(p0, m) << '\n';
  std::cout << world.total_distance(p1, m) << '\n';
  std::vector<Particle> particles;
  for (int i = 0; i < 10; i++) {
    particles.push_back(Particle(0.1, 0.1, 0));
  }
  for (Particle p : particles) {
    std::cout << world.total_distance(p, m) << '\n';
  }
  // std::cout << world.total_distance(p2, m) << '\n';
  #endif

  #if 0
  // test localization around fixed point
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  beacons.push_back(std::make_pair(1.0, 1.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world,
                    0, 0.5, 0, 0.5,
                    0.05, 0.05, 0.05,
                    5);
  print_all_particles(pf);

  // std::vector<std::pair<double, double> > measurement;
  // measurement.push_back(std::make_pair(-0.5, -0.5));
  // pf.assign_weights(beacons);
  // print_all_particles(pf);

  for (int i = 0; i < 10; i++) {
    std::vector<std::pair<double, double> > measurement;
    measurement.push_back(std::make_pair(-0.5, -0.5));
    measurement.push_back(std::make_pair(0.5, 0.5));
    pf.assign_weights(measurement);
    print_all_particles(pf);
    pf.resample();
    print_all_particles(pf);
    pf.motion_update(0, 0, 0);
    print_all_particles(pf);
    print_particle(pf.predict());
    std::cout << "\n\n";
  }
  #endif

  #if 0
  std::vector<std::pair<double, double> > beacons;
  beacons.push_back(std::make_pair(0.0, 0.0));
  WorldModel world(beacons, 0, 1, 0, 1);
  ParticleFilter pf(world,
                    0, 0.5, 0, 0.5,
                    0, 0, 0,
                    1);
  for (int i = 0; i < 10; i++) {
    std::vector<std::pair<double, double> > measurement;
    measurement.push_back(std::make_pair(-0.5, -0.5));
    pf.assign_weights(measurement);
    print_all_particles(pf);
  }
  #endif

  #if 1
  std::mt19937 rng(0);
  // test localization around fixed point
  std::vector<std::pair<double, double> > beacons;
  for (int i = 0; i < 5; i++) {
    beacons.push_back(std::make_pair(
      ((double) rng() - rng.min()) / (rng.max() - rng.min()),
      ((double) rng() - rng.min()) / (rng.max() - rng.min())
    ));
  }
  WorldModel world(beacons, 0, 8, 0, 16);
  ParticleFilter pf(world,
                    0, 8, 0, 16,
                    0.1, 0.1, 0.1,
                    200);

  // std::vector<std::pair<double, double> > measurement;
  // measurement.push_back(std::make_pair(-0.5, -0.5));
  // pf.assign_weights(beacons);
  // print_all_particles(pf);
  std::vector<std::pair<double, double> > measurement;
  for (std::pair<double, double> b : beacons) {
    measurement.push_back(std::make_pair(b.first - 3, b.second - 5));
  }
  for (int i = 0; i < 100; i++) {
    pf.assign_weights(measurement);
    pf.resample();
    pf.motion_update(0, 0, 0);
    print_particle(pf.predict());
    std::cout << "\n";
  }
  #endif

  return 0;
}
