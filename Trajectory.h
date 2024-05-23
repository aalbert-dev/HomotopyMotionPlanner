#include <vector>
#include "Pose.h"

class Trajectory {
    public:
        Trajectory();

        Trajectory(std::vector<Pose> traj);

        std::vector<Pose> getPoses() const { return poses_; }

    private:
        std::vector<Pose> poses_;
};