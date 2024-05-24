#include "Pose.h"
#include <vector>
#ifndef NODE_H
#define NODE_H
class Node{
    public:
        Node(Pose p, float time, float distance);
        Pose getPose() const { return pose_; }
        float getTime() const { return totalTime_; }
        float getDistance() const { return distance_; }
        bool operator<(const Node& other);
        // Node getPrev() const { return prev_; }
        // Node setPrev(Node prev) { prev_ = prev; }
        std::vector<Node> neighbors_;
        Node* prev_;
        bool traversed_ = false;
    
    private:
        Pose pose_;
        float totalTime_;
        float distance_;
};
#endif