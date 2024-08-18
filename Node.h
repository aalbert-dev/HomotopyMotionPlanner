#include "Pose.h"
#include <vector>
#ifndef NODE_H
#define NODE_H
class Node{
    public:
        Node(Pose p, float time, float distance);
        Pose getPose() { return pose_; }
        float getTime() const { return totalTime_; }
        float getDistance() const { return distance_; }
        bool operator<(const Node& other);
        Node* getPrev() const{ return prev_; }
        void setPrev(Node prev) { prev_ = &prev; }
        std::vector<Node> neighbors_;
        
        bool traversed_ = false;
        bool has_prev_ = false;

        double prev_x_;
        double prev_y_;
    
    private:
        Pose pose_;
        float totalTime_;
        float distance_;
        Node* prev_;

};
#endif