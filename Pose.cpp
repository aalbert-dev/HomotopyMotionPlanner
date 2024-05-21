#include "Pose.h"

#include <sstream> // for stringstream

Pose::Pose() : x_(0.0), y_(0.0), heading_(0.0) {}

Pose::Pose(double x, double y, double heading) : x_(x), y_(y), heading_(heading) {}

std::string Pose::toString() const {
  std::stringstream ss;
  ss << "Pose(x: " << x_ << ", y: " << y_ << ", heading: " << heading_ << " radians)";
  return ss.str();
}