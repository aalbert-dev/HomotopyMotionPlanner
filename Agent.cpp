#include "Agent.h"

#include <sstream> // for stringstream

Agent::Agent() : pose_(Pose()), velocity_(0.0), width_(2.0), length_(4.0){}

Agent::Agent(Pose pose, float velocity, float width, float length) : pose_(pose), velocity_(velocity), width_(width), length_(length) {}