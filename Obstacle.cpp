#include "Obstacle.h"
#include <vector>

Obstacle::Obstacle(Pose pose, float width, float length, std::string name) : pose_(pose), width_(width), length_(length), name_(name){}