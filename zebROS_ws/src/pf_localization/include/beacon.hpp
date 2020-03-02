#ifndef BEACON_HEADER
#define BEACON_HEADER
#include <string>

struct Beacon {
  const double x_;
  const double y_;
  const std::string type_;
  Beacon(double x, double y, std::string type) : x_(x), y_(y), type_(type) {}
};

#endif
