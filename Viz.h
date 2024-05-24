#include "matplotlibcpp.h"
#include <math.h>
#include "Trajectory.h"
#include "Obstacle.h"
#include "Node.h"
#include "Agent.h"

#ifndef VIZ_H
#define VIZ_H

namespace plt = matplotlibcpp;

void plot_vector(float x, float y, float heading);

void plot_obstacle(Obstacle obst);

void plot_obstacles(std::vector<Obstacle> obstacles);

void plot_agent(Agent agent);

void plot_agents(std::vector<Agent> agents);

void plot_goal(Vehicle vehicle);

void plot_traj(Trajectory traj);

void plot_pose(Pose p);

void plot_env(std::vector<Agent> agents, std::vector<Obstacle> obstacles, Vehicle vehicle, std::vector<Node> nodeGraph, Trajectory trajResult);

void plot_nodes(std::vector<Node> nodes);

#endif