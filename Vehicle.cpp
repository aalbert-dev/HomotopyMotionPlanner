#include <iostream>
#include <cmath>
#include "Pose.h"

class Vehicle {
    public:

        float mass; //kg
        float moment_of_inertia; // kg*m*m
        float wheelbase; // m
        float track_width; // m
        Pose location; // Pose
        float wheel_radius; // m
        float friction_coefficent; // 0-1
        float velocity; // m/s
        float max_motor_torque = 250; //Nm
        float motor_efficency = 0.9; // 0-1

        Vehicle(float mass, float moment_of_inertia, float wheelbase, float track_width, float wheel_radius, float friction_coefficent){
            this->mass = mass;
            this->moment_of_inertia = moment_of_inertia;
            this->track_width = track_width;
            this->wheel_radius = wheel_radius;
            this->friction_coefficent = friction_coefficent;
            Pose start_location(0, 0, 0);
            this->location = start_location;
        }

};