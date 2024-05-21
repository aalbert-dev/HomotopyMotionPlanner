#ifndef POSE_H
#define POSE_H
#include <string>

class Pose {
public:
  // Default constructor
  Pose();

  // Constructor with initial values
  Pose(double x, double y, double heading);

  // Getters for x, y, and heading
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }

  // Setters for x, y, and heading
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setHeading(double heading) { heading_ = heading; }

  // Function to get a string representation of the Pose
  std::string toString() const;

private:
  double x_;
  double y_;
  double heading_; // Radians
};

#endif // POSE_H