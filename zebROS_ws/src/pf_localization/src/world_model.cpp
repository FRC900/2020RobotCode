#include "world_model.hpp"
#include <cmath>
#include <random>
#include <utility>
#include <algorithm>
#include <map>
#include <iostream>
#include <string>

#include <ros/ros.h>


WorldModel::WorldModel(const std::vector<Beacon>& beacons,
                       double x_min, double x_max, double y_min, double y_max) :
  beacons_(beacons), boundaries_ {x_min, x_max, y_min, y_max} {}

std::vector<double> WorldModel::get_boundaries() {
  std::vector<double> res;
  for (size_t i = 0; i < 4; i++) {
    res.push_back(boundaries_[i]);
  }
  return res;
}

//checks if a given particle is within the defined boundaries
bool WorldModel::is_in_world(const Particle& p) const {
  if(p.x_ < boundaries_[0] || p.x_ > boundaries_[1]){
    return false;
  }
  if(p.y_ < boundaries_[2] || p.y_ > boundaries_[3]){
    return false;
  }
  return true;
}

//moves a given particle to the nearest position that is within the defined boundaries
void WorldModel::constrain_to_world(Particle& p) const {
  p.x_ = std::min(boundaries_[1], std::max(boundaries_[0], p.x_));
  p.y_ = std::min(boundaries_[3], std::max(boundaries_[2], p.y_));
}

//computes distances from a position a set of beacon positions
std::vector<double> WorldModel::distances(const Beacon& m,
                                          const std::vector<Beacon>& rel) const {
  std::vector<double> res;
  res.reserve(beacons_.size());
  for (const Beacon& b : rel) {
    res.push_back(hypot(m.x_ - b.x_, m.y_ - b.y_));
  }
  return res;
}

//gets the coordinates of all the beacons relative to a give particle
std::vector<Beacon> WorldModel::particle_relative(const Particle& p) const {
  std::vector<Beacon> res;
  for (const Beacon& b : beacons_) {
    double x = b.x_ - p.x_;
    double y = b.y_ - p.y_;
    double r = hypot(x, y);
    double theta = atan2(y, x);
    theta -= p.rot_;
    x = r * cos(theta);
    y = r * sin(theta);
    Beacon new_beacon {x, y, b.type_};
    res.push_back(new_beacon);
  }
  return res;
}

std::vector<Beacon> WorldModel::of_type(const std::vector<Beacon>& bcns, std::string type) {
  std::vector<Beacon> res;
  for (Beacon b : bcns) {
    if (b.type_ == type) {
      res.push_back(b);
    }
  }
  return res;
}

//Uses hungarian algorithm to pair particle relative beacons and robot relative beacons and returns the total error (sum of distance errors from particle to robot beacons)
double WorldModel::total_distance(const Particle& p, const std::vector<Beacon>& m) {
  // std::vector<int> assignment;
  // assignment.reserve(m.size());
  // std::vector<std::vector<double> > dists;
  // dists.reserve(m.size());
  // std::vector<Beacon> rel = particle_relative(p);
  // for (const Beacon& b : m) {
  //   dists.push_back(distances(b, rel));
  // }
  // solver_.Solve(dists, assignment);
  // double res = 0;
  // for (size_t i = 0; i < assignment.size(); i++) {
  //   if (assignment[i] < 0) continue;
  //   res += dists[i][assignment[i]];
  // }

  double total_res = 0;
  std::map<std::string, std::vector<Beacon> > by_type;
  for (Beacon b : m) {
    if (by_type.count(b.type_) == 0) {
      by_type[b.type_] = std::vector<Beacon>();
    }
    by_type[b.type_].push_back(b);
  }
  for (std::pair<std::string, std::vector<Beacon> > m_of_type : by_type) {
    std::vector<int> assignment;
    std::vector<std::vector<double> > dists;
    std::vector<Beacon> rel = of_type(particle_relative(p), m_of_type.first);
    for (const Beacon& b : m_of_type.second) {
      dists.push_back(distances(b, rel));
    }
    solver_.Solve(dists, assignment);
    double res = 0;
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] < 0) continue;
      res += dists[i][assignment[i]];
    }
    total_res += res;
  }

  return total_res;
}
