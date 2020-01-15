#include "world_model.hpp"
#include <cmath>
#include <random>
#include <utility>
#include <algorithm>

#include <iostream>


WorldModel::WorldModel(std::vector<std::pair<double, double> > beacons,
                       double x_min, double x_max, double y_min, double y_max) {
  beacons_ = beacons;
  boundaries_[0] = x_min;
  boundaries_[1] = x_max;
  boundaries_[2] = y_min;
  boundaries_[3] = y_max;
}

std::vector<double> WorldModel::get_boundaries() {
  std::vector<double> res;
  for (size_t i = 0; i < 4; i++) {
    res.push_back(boundaries_[i]);
  }
  return res;
}

bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x < boundaries_[0] || p.x > boundaries_[1]){
    return false;
  }
  if(p.y < boundaries_[2] || p.y > boundaries_[3]){
    return false;
  }
  return true;
}

void WorldModel::constrain_to_world(Particle& p) const {
  p.x = std::min(boundaries_[1], std::max(boundaries_[0], p.x));
  p.y = std::min(boundaries_[3], std::max(boundaries_[2], p.y));
}

//computes distances from a position a set of beacon positions
std::vector<double> WorldModel::distances(const std::pair<double, double> m,
                                          const std::vector<std::pair<double, double> > rel) const {
  std::vector<double> res;
  res.reserve(beacons_.size());
  for (std::pair<double, double> b : rel) {
    res.push_back(hypot(m.first - b.first, m.second - b.second));
  }
  return res;
}

//gets the coordinates of all the beacons relative to a give particle
std::vector<std::pair<double, double> > WorldModel::particle_relative(const Particle& p) const {
  std::vector<std::pair<double, double> > res;
  for (std::pair<double, double> b : beacons_) {
    double r = hypot(b.first - p.x, b.second - p.y);
    double theta = atan2(b.second - p.y, b.first - p.x);
    theta -= p.rot;
    double x = r * cos(theta);
    double y = r * sin(theta);
    res.push_back(std::make_pair(x, y));
  }
  return res;
}

double WorldModel::total_distance(const Particle& p, const std::vector<std::pair<double, double> >& m) {
  std::vector<int> assignment;
  assignment.reserve(m.size());
  std::vector<std::vector<double> > dists;
  dists.reserve(m.size());
  std::vector<std::pair<double, double> > rel = particle_relative(p);
  for (std::pair<double, double> b : m) {
    dists.push_back(distances(b, rel));
  }
  solver_.Solve(dists, assignment);

  double res;
  res = 0;
  for (size_t i = 0; i < assignment.size(); i++) {
    res += dists[i][assignment[i]];
  }
  return res;
}
