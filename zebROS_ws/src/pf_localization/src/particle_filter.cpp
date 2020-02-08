#include "particle_filter.hpp"
#include "world_model.hpp"
#include <random>
#include <vector>
#include <utility>

#include <iostream>
#include <ros/ros.h>


ParticleFilter::ParticleFilter(WorldModel w,
                               double x_min, double x_max, double y_min, double y_max,
                               double ns, double rs, size_t n) :
                               world_(w), num_particles_(n),
                               noise_stdev_(ns), rot_noise_stdev_(rs) {
  rng_ = std::mt19937(0);
  init(x_min, x_max, y_min, y_max);
}

void ParticleFilter::constrain_particles() {
  for (Particle& p : particles_) {
    world_.constrain_to_world(p);
  }
}

void ParticleFilter::init(double x_min, double x_max, double y_min, double y_max) {
  particles_.reserve(num_particles_);
  std::vector<double> bounds = world_.get_boundaries();
  double x_l = std::max(x_min, bounds[0]);
  double x_u = std::min(x_max, bounds[1]);
  double y_l = std::max(y_min, bounds[2]);
  double y_u = std::min(y_max, bounds[3]);
  for (size_t i = 0; i < num_particles_; i++) {
    std::uniform_real_distribution<double> x_distribution(x_l, x_u);
  	std::uniform_real_distribution<double> y_distribution(y_l, y_u);
    std::uniform_real_distribution<double> rot_distribution(0, 6.283);
    double x = x_distribution(rng_);
    double y = y_distribution(rng_);
    double rot = rot_distribution(rng_);
    Particle p = {x, y, rot};
    particles_.push_back(p);
  }
  normalize();
}

//Normalize weights to sum to 1
void ParticleFilter::normalize() {
  double sum = 0;
  for (Particle p : particles_) {
    sum += p.weight;
  }
  for (Particle& p : particles_) {
    p.weight /= sum;
  }
}


//adds random noise to particles
void ParticleFilter::noise() {
  std::normal_distribution<double> pos_dist(0, noise_stdev_);
  std::normal_distribution<double> rot_dist(0, rot_noise_stdev_);
  for (Particle& p : particles_) {
    p.x += pos_dist(rng_);
    p.y += pos_dist(rng_);
    p.rot += rot_dist(rng_);
  }
}

//Particles with higher weights are more likely to be resampled
void ParticleFilter::resample() {
  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles_);
  for (int i = 0; i < num_particles_; i++) {
    double r = ((double) rng_() - rng_.min()) / (rng_.max() - rng_.min());
    // std::cout << r << '\n';
    double a = 0;
    for (Particle p : particles_) {
      a += p.weight;
      if (a > r) {
        new_particles.push_back(p);
        break;
      }
    }
  }
  particles_ = new_particles;
}

Particle ParticleFilter::predict() {
  double weight = 0;
  Particle res {0, 0, 0};
  double s = 0;
  double c = 0;
  for (Particle& p : particles_) {
    res.x += p.x * p.weight;
    res.y += p.y * p.weight;
    c += cos(p.rot) * p.weight;
    c += sin(p.rot) * p.weight;
    weight += p.weight;
  }
  res.x /= weight;
  res.y /= weight;
  c /= weight;
  s /= weight;
  res.rot = atan2(s, c);
  return res;
}

void ParticleFilter::motion_update(double delta_x, double delta_y, double delta_rot) {
  for (Particle& p : particles_) {
    // p.x += delta_x;
    // p.y += delta_y;
    p.rot += delta_rot;
    p.x += delta_x * cos(p.rot) + delta_y * sin(p.rot);
    p.y += delta_x * sin(p.rot) + delta_y * cos(p.rot);
  }
  noise();
  constrain_particles();
}

void ParticleFilter::set_rotation(double rot) {
  for (Particle& p : particles_) {
    p.rot = rot;
  }
}

//assigns the reciprocal of the computed error of each particles assignment vector to the respective particle
void ParticleFilter::assign_weights(std::vector<std::pair<double, double> > mBeacons) {
  for (Particle& p : particles_) {
    p.weight = 1 / world_.total_distance(p, mBeacons);
  }
  normalize();
}

std::vector<Particle> ParticleFilter::get_particles() const {
  return particles_;
}
