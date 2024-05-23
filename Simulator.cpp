#include "Vehicle.h"
#include "Logger.cpp"
#include "Pose.h"
#include <iostream>
#include <math.h>
#include "Agent.h"
#include "Viz.cpp"

void updateKinematics(Vehicle& car, float dt, float steering_angle, float throttle){
    
    // // Calculate turning radius and wheel angles 
    // double turning_radius = car.wheelbase / tan(steering_angle);
    // double delta_inner = atan2(car.wheelbase, turning_radius + car.wheelbase / 2);
    // double delta_outer = atan2(car.wheelbase, turning_radius - car.wheelbase / 2);

    // // Calculate engine force
    // double motor_torque = throttle * car.max_motor_torque;
    // double engine_force = motor_torque * car.motor_efficency / car.wheel_radius;

    // // Calculate friction force
    // double friction_force = car.friction_coefficent * car.mass * car.velocity;

    // // Calculate net force and acceleration
    // double net_force = engine_force - friction_force;
    // double acceleration = net_force / car.mass;

    // // Update velocity
    // double new_velocity = car.velocity + acceleration * dt;
    // car.velocity = new_velocity;

    // // Calculate angular velocity
    // double omega = new_velocity / turning_radius;
    // std::cout << turning_radius << std::endl;

    // // Calculate new heading and pose 
    // double new_heading = car.location.getHeading() + omega * dt;
    // double delta_x = new_velocity * cos(car.location.getHeading()) * dt;
    // double delta_y = new_velocity * sin(car.location.getHeading()) * dt;

    // // Update car pose
    // car.location.setX(car.location.getX() + delta_x);
    // car.location.setY(car.location.getY() + delta_y);
    // car.location.setHeading(new_heading);

}

int main(int argc, char **argv){
    Agent test_agent1(Pose(0, 0, M_PI/4), 2.0, 2.0, 4.0);
    Agent test_agent2(Pose(-5, 1, M_PI/4), 2.0, 2.0, 4.0);
    Vehicle ego_vehicle;
    std::vector<Agent> agents;
    agents.push_back(test_agent1);
    agents.push_back(test_agent2);
    plot_env(agents, ego_vehicle);
    return 0;
}