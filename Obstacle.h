#include <vector>
#include <sstream>

#ifndef OBST_H
#define OBST_H
class Obstacle {
    public:

        Obstacle(std::vector<float> points_x, std::vector<float> points_y, std::string name);

        std::vector<float> getPointsX() const { return points_x_; }
        std::vector<float> getPointsY() const { return points_y_; }

        std::string name_;

    private:
        std::vector<float> points_x_;
        std::vector<float> points_y_;
};

#endif