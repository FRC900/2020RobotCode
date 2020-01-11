#ifndef WORLD_MODEL_HEADER
#define WORLD_MODEL_HEADER

class WorldModel {
private:
  std::vector<std::pair<double, double> > beacons_;
  double boundaries_[4]; // xmin, xmax, ymin, ymax

public:
  WorldModel();
  bool is_in_world(const Particle& p) const;
  void init_particle(Particle& p, double x, double y, double stdev);
};

#endif
