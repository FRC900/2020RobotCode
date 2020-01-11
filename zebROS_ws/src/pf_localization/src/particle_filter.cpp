#include "particle_filter.hpp"
#include "world_model.hpp"
#include <random>
#include <vector>

void ParticleFilter::init(double x, double y) {
  for (Particle p : particles_) {
    world_.init_particle(p, x, y, init_stdev);
  }
}

void ParticleFilter::noise() {
  std::normal_distribution<double> pos_dist(0, noise_stdev_);
  std::normal_distribution<double> rot_dist(0, rot_noise_stdev_);
  for (Particle p : particles_) {
    p.x += pos_dist(rng_);
    p.y += pos_dist(rng_);
    p.rot += rot_dist(rng_);
  }
}

void ParticleFilter::resample() {
  std::vector<Particle> new_particles;
}
