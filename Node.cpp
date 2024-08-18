#include "Node.h"

Node::Node(Pose p, float time, float distance) : pose_(p), totalTime_(time), distance_(distance){}
// Node::Node() : pose_(Pose(0, 0, 0)), totalTime_(0), distance_(0){}

bool Node::operator<(const Node& other){
            return this->getDistance() > other.getDistance();
        }