#include "matplotlibcpp.h"
#include <math.h>

namespace plt = matplotlibcpp;

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
    float agent_top_center_x1 = 0;
    float agent_top_center_y1 = agent_length / 2;
    float agent_top_center_x2 = 0;
    float agent_top_center_y2 = agent_length / 1.5;

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

    // Rotate points given heading
    float s = sin(agent_heading);
    float c = cos(agent_heading);

    for (int i = 0; i < points_x.size(); i++){
        float new_x = points_x[i] * c - points_y[i] * s;
        float new_y = points_x[i] * s + points_y[i] * c;
        points_x[i] = new_x + agent_pose_x;
        points_y[i] = new_y + agent_pose_y;
    }

    // Calculate arrow vector points
    float new_top_center_x1 = agent_top_center_x1 * c - agent_top_center_y1 * s + agent_pose_x;
    float new_top_center_y1 = agent_top_center_x1 * s + agent_top_center_y1 * c + agent_pose_y;
    float new_top_center_x2 = agent_top_center_x2 * c - agent_top_center_y2 * s + agent_pose_x;
    float new_top_center_y2 = agent_top_center_x2 * s + agent_top_center_y2 * c + agent_pose_y;

    // Plot agent points
    plt::plot(points_x, points_y);

    // Plot agent vector
    plt::arrow(new_top_center_x1, new_top_center_y1, new_top_center_x2, new_top_center_y2);
    
}

void plot_agents(std::vector<Agent> agents){
    for (int i = 0; i < agents.size(); i++){
        plot_agent(agents[i]);
    }
}

void plot_env(std::vector<Agent> agents, Vehicle vehicle){
    plt::figure_size(1920, 1080);
    plot_agents(agents);
    plt::save("./basic.png");
}