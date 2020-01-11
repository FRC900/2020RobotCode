#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

#include "particle.hpp"
#include "hungarian.hpp"

class WorldModel {
private:
  std::vector<std::pair<double, double> > beacons_;
  double boundaries_[4]; // xmin, xmax, ymin, ymax
  AssignmentProblemSolver<double> solver_;

public:
  WorldModel();
  bool is_in_world(const Particle& p) const;
  void init_particle(Particle& p, double x, double y, double stdev);
  double total_distance(const Particle& p, const std::vector<std::pair<double, double> >& m);
  std::vector<double> distances(const std::pair<double, double> m) const;
  std::vector<std::pair<double, double> > particle_relative(const Particle& p) const;
};

#endif
