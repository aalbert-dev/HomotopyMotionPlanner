#include "Pose.h"
#include <vector>
#ifndef NODE_H
#define NODE_H
class Node{
    public:
        Node(Pose p);
        Pose getPose() const { return pose_; }
        std::vector<Node> neighbors_;
    
    private:
        Pose pose_;
};
#endif