#include <vector>
#include <sstream>
#include "Pose.h"

#ifndef OBST_H
#define OBST_H
class Obstacle {
    public:

        Obstacle(Pose p, float width, float length, std::string name);

        Pose getPose() const { return pose_; }
        float getWidth() const { return width_; }
        float getLength() const { return length_; }

        std::string name_;

    private:
        Pose pose_;
        float width_;
        float length_;
};

#endif