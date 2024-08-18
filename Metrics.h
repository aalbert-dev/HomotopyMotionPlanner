#include "Agent.h"
#include <vector> 

class Metrics{
    public:
        Metrics(std::vector<Agent> agents);
        double timeToCollision(Agent& agent1, Agent& agent2);
        double postEncroachmentTime(Agent& agent1, Agent& agent2);
    private:
        std::vector<Agent> agents_;

};