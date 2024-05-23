#include <iostream>
#include <cmath>
#include "Agent.h"
#include <vector>

class Vehicle : public Agent{
    public:
        Vehicle(Pose pose, float velocity, float width, float length, std::string name, float goal_x, float goal_y);
        void calculateTrajectory(std::vector<Agent> agents);
        float getGoalX() const { return goal_x_; }
        float getGoalY() const { return goal_y_; }

    private:
        float goal_x_;
        float goal_y_;
};