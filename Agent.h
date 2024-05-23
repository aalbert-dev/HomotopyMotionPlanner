#include <iostream>
#include <cmath>
#include "Pose.h"
#ifndef AGENT_H
#define AGENT_H
class Agent {
    public:

        // Default constructor
        Agent();

        // Constructor with initial values
        Agent(Pose pose, float velocity, float width, float length);

        // Getter functions
        Pose getPose() const { return pose_; }
        float getVeloity() const { return velocity_; }
        float getWidth() const { return width_; }
        float getLength() const { return length_; }

    private:
        Pose pose_;
        float velocity_;
        float width_;
        float length_;
};
#endif