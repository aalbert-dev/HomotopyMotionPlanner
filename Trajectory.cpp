#include "Trajectory.h"

Trajectory::Trajectory() : poses_(std::vector<Pose> {Pose()}){}

Trajectory::Trajectory(std::vector<Pose> poses) : poses_(poses){}