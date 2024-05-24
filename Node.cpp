#include "Node.h"

Node::Node(Pose p, float time, float distance) : pose_(p), totalTime_(time), distance_(distance){}

bool Node::operator<(const Node& other){
            return this->getDistance() > other.getDistance();
        }