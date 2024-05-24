#include <vector>
#include "Pose.h"
#ifndef TRAJ_H
#define TRAJ_H
class Trajectory {
    public:
        Trajectory();

        Trajectory(std::vector<Pose> traj);

        std::vector<Pose> getPoses() const { return poses_; }

    private:
        std::vector<Pose> poses_;
};

#endif