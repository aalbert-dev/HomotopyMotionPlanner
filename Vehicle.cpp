#include "Vehicle.h"

Vehicle::Vehicle(Pose pose, float velocity, float width, float length, std::string name, float goal_x, float goal_y)
      : Agent(pose, velocity, width, length, name), goal_x_(goal_x), goal_y_(goal_y) {}

void Vehicle::calculateTrajectory(std::vector<Agent> agents){
    
}