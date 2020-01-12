/*
just a main function to test if ParticleFilter builds
*/

#include "particle_filter.hpp"
#include "world_model.hpp"

int main(int argc, char const *argv[]) {
  WorldModel world(0, 0, 1, 1);
  ParticleFilter pf(world, 0, 0, 0.5, 0.5, 0.1, 0.1, 0.1, 100);
  return 0;
}
