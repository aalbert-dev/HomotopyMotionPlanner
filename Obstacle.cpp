#include "Obstacle.h"
#include <vector>

Obstacle::Obstacle(std::vector<float> points_x, std::vector<float> points_y, std::string name) : points_x_(points_x), points_y_(points_y), name_(name) {}