#ifndef PARTICLE_FILTER_HEADER
#define PARTICLE_FILTER_HEADER

#include <utility>
#include <random>

struct Particle {
  double weight;
  double x;
  double y;
  double rot;

  Particle(double x, double y, double rot): x(x), y(y), rot(rot), weight(0) {}
};

class ParticleFilter {
private:
  double init_stdev_ = 1;
  double noise_stdev_ = 1;
  double rot_noise_stdev_ = 1;
  std::mt19937 rng_(0);
  std::vector<Particle> particles_;
  WorldModel world_;
  void resample();
  void noise();
  void init(double x, double y);

public:
  ParticleFilter(WorldModel w, double x, double y, int num_particles);
  double predict();
  void motion_update();
  void particle_iter(std::vector<std::pair<double, double> > mBeacons);
};


#endif
