#include "matplotlibcpp.h"
#include <math.h>
#include "Trajectory.h"
#include "Agent.h"
#include "Vehicle.h"
#include "Obstacle.h"
#include "Node.h"

namespace plt = matplotlibcpp;

void plot_vector(float x, float y, float heading){
    float arrow_length = 0.5;
    float dx = cos(heading);
    float dy = sin(heading);
    plt::arrow(x, y, dx * arrow_length, dy * arrow_length);
}

void plot_obstacle(Obstacle obst){
    std::vector<float> points_x;
    std::vector<float> points_y;

    // get Agent dimensions
    Pose obst_pose = obst.getPose();
    float obst_pose_x = obst_pose.getX();
    float obst_pose_y = obst_pose.getY();
    float obst_heading = obst_pose.getHeading();
    float obst_width = obst.getWidth();
    float obst_length = obst.getLength();

    // Calculate agent points
    // Top left
    float obst_x1 = obst_pose_x + obst_width / 2;
    float obst_y1 = obst_pose_y + obst_length / 2;
    // Top right
    float obst_x2 = obst_pose_x - obst_width / 2;
    float obst_y2 = obst_pose_y + obst_length / 2;
    // Bottom right
    float obst_x3 = obst_pose_x - obst_width / 2;
    float obst_y3 = obst_pose_y - obst_length / 2;
    // Bottom left
    float obst_x4 = obst_pose_x + obst_width / 2;
    float obst_y4 = obst_pose_y - obst_length / 2;

    // Add points to vector
    points_x.push_back(obst_x1);
    points_x.push_back(obst_x2);
    points_x.push_back(obst_x3);
    points_x.push_back(obst_x4);
    //Add final x to close rectangle
    points_x.push_back(obst_x1);
    
    points_y.push_back(obst_y1);
    points_y.push_back(obst_y2);
    points_y.push_back(obst_y3);
    points_y.push_back(obst_y4);
    //Add final y to close rectangle
    points_y.push_back(obst_y1);

    // Transform points to origin 
    for (int i = 0; i < points_x.size(); i++){
        points_x[i] -= obst_pose_x;
    }
    for (int i = 0; i < points_y.size(); i++){
        points_y[i] -= obst_pose_y;
    }
        
    float s = sin(obst_heading);
    float c = cos(obst_heading);

    for (int i = 0; i < points_x.size(); i++){
        float new_x = points_x[i] * c - points_y[i] * s;
        float new_y = points_x[i] * s + points_y[i] * c;
        points_x[i] = new_x + obst_pose_x;
        points_y[i] = new_y + obst_pose_y;
    }
    // Plot agent points
    plt::plot(points_x, points_y);
    plt::text(obst_pose_x, obst_pose_y, obst.name_);
}

void plot_obstacles(std::vector<Obstacle> obstacles){
    for (int i = 0; i < obstacles.size(); i++){
        plot_obstacle(obstacles[i]);
    }
}

void plot_agent(Agent agent){
    std::vector<float> points_x;
    std::vector<float> points_y;

    // get Agent dimensions
    Pose agent_pose = agent.getPose();
    float agent_pose_x = agent_pose.getX();
    float agent_pose_y = agent_pose.getY();
    float agent_heading = agent_pose.getHeading();
    float agent_width = agent.getWidth();
    float agent_length = agent.getLength();

    // Calculate agent points
    // Top left
    float agent_x1 = agent_pose_x + agent_width / 2;
    float agent_y1 = agent_pose_y + agent_length / 2;
    // Top right
    float agent_x2 = agent_pose_x - agent_width / 2;
    float agent_y2 = agent_pose_y + agent_length / 2;
    // Bottom right
    float agent_x3 = agent_pose_x - agent_width / 2;
    float agent_y3 = agent_pose_y - agent_length / 2;
    // Bottom left
    float agent_x4 = agent_pose_x + agent_width / 2;
    float agent_y4 = agent_pose_y - agent_length / 2;
    // Top center points for agent direction vector 
    float agent_top_center_x = agent_pose_x + agent_width / 2;
    float agent_top_center_y = agent_pose_y;

    // Add points to vector
    points_x.push_back(agent_x1);
    points_x.push_back(agent_x2);
    points_x.push_back(agent_x3);
    points_x.push_back(agent_x4);
    //Add final x to close rectangle
    points_x.push_back(agent_x1);
    
    points_y.push_back(agent_y1);
    points_y.push_back(agent_y2);
    points_y.push_back(agent_y3);
    points_y.push_back(agent_y4);
    //Add final y to close rectangle
    points_y.push_back(agent_y1);

    // Transform points to origin 
    for (int i = 0; i < points_x.size(); i++){
        points_x[i] -= agent_pose_x;
    }
    for (int i = 0; i < points_y.size(); i++){
        points_y[i] -= agent_pose_y;
    }
        
    float s = sin(agent_heading);
    float c = cos(agent_heading);

    for (int i = 0; i < points_x.size(); i++){
        float new_x = points_x[i] * c - points_y[i] * s;
        float new_y = points_x[i] * s + points_y[i] * c;
        points_x[i] = new_x + agent_pose_x;
        points_y[i] = new_y + agent_pose_y;
    }
    // Plot agent points
    plt::plot(points_x, points_y);
    plt::text(agent_pose_x, agent_pose_y, agent.name_);
    // Plot agent vector
    plot_vector(agent_top_center_x, agent_top_center_y, agent_heading);
}

void plot_agents(std::vector<Agent> agents){
    for (int i = 0; i < agents.size(); i++){
        plot_agent(agents[i]);
    }
}

void plot_goal(Vehicle vehicle){
    std::vector<float> goal_x;
    std::vector<float> goal_y;
    goal_x.push_back(vehicle.getGoalX());
    goal_y.push_back(vehicle.getGoalY());
    plt::scatter(goal_x, goal_y, 50);
}

void plot_traj(Trajectory traj){
    std::vector<Pose> poses = traj.getPoses();
    for (int i = 0; i < poses.size(); i++){
        Pose p = poses[i];
        plot_vector(p.getX(), p.getY(), p.getHeading());
    }
}

void plot_pose(Pose p){
    float x1 = p.getX();
    float y1 = p.getY();
    float heading = p.getHeading();
}

void plot_node(Node node){
    if (node.has_prev_){
        float x1 = node.getPose().getX();
        float y1 = node.getPose().getY();
        // float x2 = node.getPrev()->getPose().getX();
        // float y2 = node.getPrev()->getPose().getY();
        float x2 = node.prev_x_;
        float y2 = node.prev_y_;
        std::vector<float> points_x = {x1, x2};
        std::vector<float> points_y = {y1, y2};
        plt::plot(points_x, points_y);
        //std::cout << "plotting" << std::endl;
    }
}

void plot_nodes(std::vector<Node> nodes){
    for (int i = 0; i < nodes.size(); i++){
        plot_node(nodes[i]);
    }
}

void plot_env(std::vector<Agent> agents, std::vector<Obstacle> obstacles, Vehicle vehicle, std::vector<Node> nodeGraph, Trajectory trajResult){
    plt::figure_size(1920, 1080);
    plt::xlim(-16, 16);
    plt::ylim(-16, 16);
    plot_agents(agents);
    plot_agent(vehicle);
    plot_goal(vehicle);
    plot_obstacles(obstacles);
    //plot_traj(trajResult);
    plot_nodes(nodeGraph);
    // for (int i = 0; i < nodeGraph.size(); i++){
    //     double x2 = nodeGraph[i].getPrev()->getPose().getX();
    //     double y2 = nodeGraph[i].getPrev()->getPose().getY();
    //     std::cout << x2 << " " << y2 << std::endl;
    // }
    
    plt::save("./basic.png");
}