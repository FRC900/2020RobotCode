#include "particle_filter.hpp"
#include "world_model.hpp"
#include <random>
#include <vector>
#include <utility>


void ParticleFilter::init(double x, double y) {
  for (Particle p : particles_) {
    world_.init_particle(p, x, y, init_stdev);
  }
}

void ParticleFilter::normalize() {
  double sum = 0;
  for (Particle p : particles_) {
    sum += p.weight;
  }
  for (Particle p : particles_) {
    p.weight /= sum;
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
  new_particles.reserve(num_particles_);
  for (int i = 0; i < num_particles_; i++) {
    double r = (rng_() - rng_.min()) / (rng_.max() - rng.min());
    double a = 0;
    for (Particle p : particles) {
      a += p.weight;
      if (a > r) {
        new_particles.push_back(p);
        break;
      }
    }
  }
  particles_ = new_particles;
}

ParticleFilter(WorldModel w, double x, double y, int n);

Particle ParticleFilter::predict() {
  return Particle(0, 0, 0); // TODO
}

void motion_update() {
  // TODO
}

void assign_weights(std::vector<std::pair<double, double> > mBeacons) {
  for (Particle p : particles) {
    p.weight = world_.total_distance(p, mBeacons);
  }
  normalize();
}
