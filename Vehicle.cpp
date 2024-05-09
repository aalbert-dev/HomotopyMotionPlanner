#include <iostream>
#include "Pose.cpp"

class Vehicle {
    public:

        float mass; //kf
        float moment_of_inertia; // kg*m*m
        float wheelbase; // m
        float track_width; // m
        Pose center_of_mass; // Pose(x, y, heading)
        float wheel_radius; // m
        float friction_coefficent; // 0-1

        Vehicle(float mass, float moment_of_inertia, float wheelbase, float track_width, Pose center_of_mass, float wheel_radius, float friction_coefficent){
            this->mass = mass;
            this->moment_of_inertia = moment_of_inertia;
            this->track_width = track_width;
            this->center_of_mass = center_of_mass;
            this->wheel_radius = wheel_radius;
            this->friction_coefficent = friction_coefficent;
        }

        int simulate(float steering_angle, float throttle, float brake_force);
};

int Vehicle::simulate(float steering_angle, float throttle, float brake_force){
    // float streeing_angle rad
    // float throttle 0-1
    // float brake_force newtons
    float turning_radius = wheelbase /  tan(steering_angle)
    std::cout << "Test";
    return 0;
}