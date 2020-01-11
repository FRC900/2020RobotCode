#include <world_model.hpp>

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
