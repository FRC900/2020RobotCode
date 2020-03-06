#ifndef BEACON_HEADER
#define BEACON_HEADER
#include <string>

struct Beacon {
  const double x_;
  const double y_;
  const std::string type_;
  Beacon(double x, double y, std::string type) : x_(x), y_(y), type_(type) {}
};

struct BearingBeacon {
  const double angle_;
  const std::string type_;
  BearingBeacon(double angle, std::string type) : angle_(angle), type_(type) {}
};

#endif
