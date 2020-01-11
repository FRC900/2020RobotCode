#include "world_model.hpp"
#include <cmath>
#include <utility>


bool WorldModel::is_in_world(const Particle& p) {
  if(p.x < boundaries_[0] || p.x > boundaries_[1]){
    return false;
  }
  if(p.y < boundaries_[2] || p.y > boundaries_[3]){
    return false;
  }
  return true;
}

void WorldModel::init_particle(Particle p&, double x, double y, double stdev){
  std::uniform_real_distribution<double> x_distribution(min_x, max_x);
	std::uniform_real_distribution<double> y_distribution(min_y, max_y);
}

std::vector<double> WorldModel::distances(const std::pair<double, double> m) const {
  std::vector<double> res;
  res.reserve(beacons_.size());
  for (std::pair<double, double> b : beacons_) {
    res.append(hypot(m.first - b.first, m.second - b.second));
  }
  return res;
}

std::vector<std::pair<double, double> > WorldModel::particle_relative(const Particle& p) const {
  std::vector<std::pair<double, double> > res;
  for (std::vector<double, double> b : beacons_) {
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
  dists.reserve(mBeacons.size());
  
  solver_.solve(dists, assignment);
}
