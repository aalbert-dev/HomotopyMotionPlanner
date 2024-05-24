#include "Agent.h"

#include <sstream>

Agent::Agent() : pose_(Pose()), velocity_(0.0), width_(2.0), length_(4.0), name_(""){}

Agent::Agent(Pose pose, float velocity, float width, float length, std::string name) : pose_(pose), velocity_(velocity), width_(width), length_(length), name_(name){}