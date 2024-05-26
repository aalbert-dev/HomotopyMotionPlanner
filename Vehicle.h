#include <iostream>
#include <cmath>
#include "Agent.h"
#include "Trajectory.h"
#include "Obstacle.h"
#include <vector>
#include "Node.h"

class Vehicle : public Agent{
    public:
        Vehicle(Pose pose, float velocity, float width, float length, std::string name, float goal_x, float goal_y);
        Trajectory calculateTrajectory(std::vector<Agent> agents, std::vector<Obstacle> obstacles);
        std::vector<Node> calculateNodeGraph(std::vector<Agent> agents, std::vector<Obstacle> obstacles);
        std::vector<Node> calculateNodeGraph2(std::vector<Agent> agents, std::vector<Obstacle> obstacles);
        std::vector<Node> randomTreeExplore(std::vector<Agent> agents, std::vector<Obstacle> obstacles);
        Pose getNextPose(Pose currentPose, float angle, float distance);
        Agent projectEgo(Pose newPose);
        bool checkNextPose(std::vector<Agent> agents, std::vector<Obstacle> obstacles);
        float getPoseDistance(Pose p);
        float getEgoPoseDistance(Pose p);
        float getGoalX() const { return goal_x_; }
        float getGoalY() const { return goal_y_; }
        bool isGoal(Pose currentPose);
        Node generateGoalNode();


    private:
        float goal_x_;
        float goal_y_;
};