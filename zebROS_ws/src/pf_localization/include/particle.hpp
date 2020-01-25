#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER

struct Particle {
  double weight;
  double x;
  double y;
  double rot;

  Particle(double x, double y, double rot): x(x), y(y), rot(rot), weight(1) {}
};

#endif
