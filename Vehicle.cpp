#include "Vehicle.h"
#include "Obstacle.h"
#include "Trajectory.h"
#include "Node.h"
#include <random>
#include <math.h>
#include <bits/stdc++.h>
#include "Viz.h"
#include <cstdlib>
#include <ctime>


static std::default_random_engine rng;


Vehicle::Vehicle(Pose pose, float velocity, float width, float length, std::string name, float goal_x, float goal_y)
      : Agent(pose, velocity, width, length, name), goal_x_(goal_x), goal_y_(goal_y) {}


Pose Vehicle::getNextPose(Pose currentPose, float angle, float distance){
    float dx = cos(angle) * distance;
    float dy = sin(angle) * distance;
    return Pose(currentPose.getX() + dx, currentPose.getY() + dy, angle);
}

float distance(Pose p1, Pose p2){
    return pow(pow(p1.getX() - p2.getX(), 2) + pow(p1.getY() - p2.getY(), 2), 0.5);
}

float distance(Node n1, Node n2){
   return distance(n1.getPose(), n2.getPose());
}

bool Vehicle::isGoal(Pose currentPose){
    Pose goalPose(getGoalX(), getGoalY(), 0.0);
    return distance(currentPose, goalPose) < 1;
}

bool compare(Node n1, Node n2){
    return n1.getDistance() < n2.getDistance();
}

bool checkCollision(Agent ego, Obstacle obstacle){
    // Returns true if there is a collision
    return (ego.getPose().getX() < obstacle.getPose().getX() + obstacle.getLength() &&
          ego.getPose().getX() + ego.getLength() > obstacle.getPose().getX() &&
          ego.getPose().getY() < obstacle.getPose().getY() + obstacle.getWidth() &&
          ego.getPose().getY() + ego.getWidth() > obstacle.getPose().getY());
}

bool checkCollision(Agent agent1, Agent agent2){
    // Returns true if there is a collision
    return (agent1.getPose().getX() < agent2.getPose().getX() + agent2.getLength() &&
          agent1.getPose().getX() + agent1.getLength() > agent2.getPose().getX() &&
          agent1.getPose().getY() < agent2.getPose().getY() + agent2.getWidth() &&
          agent1.getPose().getY() + agent1.getWidth() > agent2.getPose().getY());
}

bool checkCollisions(Agent ego, std::vector<Agent> otherAgents){
    // Checks for collisions with other agents
    for (int i = 0; i < otherAgents.size(); i++){
        if (checkCollision(ego, otherAgents[i])){
            return true;
        }
    }
    return false;
}

bool checkCollisions(Agent ego, std::vector<Obstacle> obstacles){
    // Check for collisions with obstacles
    for (int i = 0; i < obstacles.size(); i++){
        if (checkCollision(ego, obstacles[i])){
            return true;
        }
    }
    return false;
}

bool checkCollisions(Agent ego, std::vector<Agent> otherAgents, std::vector<Obstacle> obstacles){
    // Returns true if there is a collision
    // Check collisions with agents
    if (checkCollisions(ego, otherAgents)){
        return true;
    }
    // Check collisions with obstacles
    if (checkCollisions(ego, obstacles)){
        return true;
    }
    return false;
}

Pose getFuturePose(Pose currentPose, float time, float velocity){
    float distance = time * velocity;
    float heading = currentPose.getHeading();
    float dx = currentPose.getX() + distance * cos(heading);
    float dy = currentPose.getY() + distance * sin(heading);
    return Pose(dx, dy, heading);

}

Agent projectAgent(Agent otherAgent, Pose newPose){
    return Agent(newPose, otherAgent.getVeloity(), otherAgent.getWidth(), otherAgent.getLength(), otherAgent.name_);
}

Agent Vehicle::projectEgo(Pose newPose){
    return Agent(newPose, getVeloity(), getWidth(), getLength(), name_);
}

Agent projectVehicle(Vehicle egoVehicle, Pose newPose){
    return projectAgent(egoVehicle, newPose);
}

std::vector<Agent> projectAgents(std::vector<Agent> otherAgents, float time){
    std::vector<Agent> predictedAgents;
    for (int i = 0; i < otherAgents.size(); i++){
        // Agent predictedAgent = ;
        predictedAgents.push_back(projectAgent(otherAgents[i], getFuturePose(otherAgents[i].getPose(), time, otherAgents[i].getVeloity())));
        // delete &predictedAgent;
    }
    return predictedAgents;
}

Pose pickRandomPose(Pose startPose, float maxTurnAngle, float maxVelocity, float dt){
    std::uniform_real_distribution<double> angleDist(-1 * maxTurnAngle, maxTurnAngle); 
    std::uniform_real_distribution<double> velocityDist(0, maxVelocity); 
    float randomAngle = startPose.getHeading() + angleDist(rng);
    float randomVelocity = velocityDist(rng);
    float startX = startPose.getX();
    float startY = startPose.getY();
    return Pose(startX + cos(randomAngle) * randomVelocity * dt, startY + sin(randomAngle) * randomVelocity * dt, randomAngle);
}

Node pickRandomNode(std::vector<Node> nodeList){
    std::uniform_int_distribution<int> nodeDist(0, nodeList.size() - 1);
    return nodeList[nodeDist(rng)];
}

// float calculateCost(Node pickedNode){
//     float total_cost = 0;
//     Node* newNode = currentNode.prev_;
//     while (newNode->prev_){
//         poseList.push_back(newNode->getPose());
//         newNode = newNode->prev_;
//         std::cout << "Next node in traj" << std::endl;
//     }
// }

Node findLowestCostNeighbor(std::vector<Node> nodeList, Node pickedNode, Node originalNode, float radius){
    float lowestCost = originalNode.getDistance() + distance(pickedNode, originalNode);
    Node lowestCostNode = originalNode;
    for (int i = 0; i < nodeList.size(); i++){
        float newDistance = distance(nodeList[i], pickedNode);
        if (newDistance < radius){
            float newCost = nodeList[i].getDistance() + newDistance;
            if (newCost < lowestCost){
                lowestCost = newCost;
                lowestCostNode = nodeList[i];
            }
        }
    }
    return lowestCostNode;
}

Node Vehicle::generateGoalNode(){
    Pose goalPose(getGoalX(), getGoalY(), 0.0);
    return Node(goalPose, 0, 0);
}

float Vehicle::getPoseDistance(Pose p){
    return pow(pow(p.getX() - getGoalX(), 2) + pow(p.getY() - getGoalY(), 2), 0.5);
}

float Vehicle::getEgoPoseDistance(Pose p){
    return pow(pow(p.getX() - getPose().getX(), 2) + pow(p.getY() - getPose().getY(), 2), 0.5);
}

float noise(){
    std::uniform_real_distribution<double> dist(0.0, 5.0); 
    return dist(rng); 
}

bool pickRandom(){
    std::uniform_int_distribution<int> dist(0, 5);
    return dist(rng) == 0;
}

int pickRandomIndex(int maxIndex){
    return maxIndex / 2;
}

std::vector<Node> Vehicle::calculateNodeGraph(std::vector<Agent> agents, std::vector<Obstacle> obstacles){
    // Create tree from graph space
    // Initialize RRT default values
    int currentTreeIter = 0;
    int maxTreeSize = 10000;
    float dt = 0.5;
    float velocity = 2.5; 
    float maxTurnAngle = M_PI / 8;
    float minTurnAngle = M_PI / 64;

    // Get first node from vehicle current pose
    Pose currentPose = getPose();
    Node currentNode(currentPose, 0, getPoseDistance(currentPose));
    std::vector<Node> nodeHeap = {currentNode};
    std::vector<Node> nodeList = {currentNode};

    // Iterate until goal is found or maxTreeSize is reached
    while (currentTreeIter < maxTreeSize){

        // Extract current node from front of heap
        std::vector<Pose> possibleNextPoses;
        int randomIndex;
        bool shouldPickRandom = pickRandom();
        if (!shouldPickRandom){
            currentNode = nodeHeap.front();
            currentPose = currentNode.getPose();
        }else{
            randomIndex = pickRandomIndex(nodeHeap.size());
            currentNode = nodeHeap[randomIndex];
            currentPose = currentNode.getPose();
        }

        // Check if current pose is within goal range ( < 1 )
        if (isGoal(currentPose)){
            return nodeList;
        }
        

        // Calculate all possible moves (turning angles)
        float currentAngle = currentPose.getHeading();
        bool freeNodes = false;
        for (float nextAngle = currentAngle - maxTurnAngle; nextAngle <= maxTurnAngle + currentAngle; nextAngle += minTurnAngle){

            // Project ego and agent into corresponding future time
            Pose potentialNextPose = getNextPose(currentPose, nextAngle, dt * velocity);
            float totalTime = currentNode.getTime() + dt;
            std::vector<Agent> predictedAgents = projectAgents(agents, totalTime);
            Agent futureEgo = projectEgo(potentialNextPose);

            // Check for collisions
            bool futureCollision = checkCollisions(futureEgo, predictedAgents, obstacles);
            if (!futureCollision){

                // Add node to heap if there aren't any collisions
                float nextNodeDistance = getPoseDistance(potentialNextPose);
                //std::cout << "NEW DISTANCE: " << nextNodeDistance << std::endl;
                Node nextNode(potentialNextPose, totalTime, nextNodeDistance);
                // currentNode.neighbors_.push_back(nextNode);
                // if (!nextNode.prev_) nextNode.prev_ = &currentNode;
                nodeHeap.push_back(nextNode);
                nodeList.push_back(nextNode);
                push_heap(nodeHeap.begin(), nodeHeap.end());
                freeNodes = true;
            }
        }
        // if (!freeNodes){
        //     std::cout << "No free nodes" << std::endl;
        // }

        // Incremnt tree iter count
        currentTreeIter++;

        // Pop from heap
        if (!shouldPickRandom){
            pop_heap(nodeHeap.begin(), nodeHeap.end());
            nodeHeap.pop_back();
        }else{
            std::swap(nodeHeap[randomIndex], nodeHeap[nodeHeap.size() - 1]);
            nodeHeap.pop_back();
            make_heap(nodeHeap.begin() + randomIndex, nodeHeap.end());
        }
        
    }
    return nodeList;
}

std::vector<Node> Vehicle::calculateNodeGraph2(std::vector<Agent> agents, std::vector<Obstacle> obstacles){
    // Create tree from graph space
    // Initialize RRT default values
    int currentTreeIter = 0;
    int maxTreeSize = 10000;
    float dt = 1;
    float min_velocity = 1; 
    float max_velocity = 5;
    float velocity_increment = 1;
    float maxTurnAngle = M_PI / 4;
    float minTurnAngle = M_PI / 32;

    // Get first node from vehicle current pose
    Pose currentPose = getPose();
    Node currentNode(currentPose, 0, getPoseDistance(currentPose));
    std::vector<Node> nodeHeap = {currentNode};
    std::vector<Node> nodeList = {currentNode};

    bool freeNodes = false;

    // Iterate until goal is found or maxTreeSize is reached
    while (currentTreeIter < maxTreeSize){

        // Extract current node from front of heap
        std::vector<Pose> possibleNextPoses;
        
        currentNode = nodeHeap.front();
        std::cout << currentNode.getDistance() << std::endl;
        if (currentNode.traversed_){
            std::cout << "Already traversed" << std::endl;
        }
        currentNode.traversed_ = true;
        currentPose = currentNode.getPose();

        // Check if current pose is within goal range ( < 1 )
        if (isGoal(currentPose)){
            return nodeList;
        }
        

        // Calculate all possible moves (turning angles)
        float currentAngle = currentPose.getHeading();
        freeNodes = false;

        // Iterate through velocities
        for (float nextVelocity = min_velocity; nextVelocity <= max_velocity; nextVelocity += velocity_increment){
            
            // Iterate through angles
            for (float nextAngle = currentAngle - maxTurnAngle; nextAngle <= maxTurnAngle + currentAngle; nextAngle += minTurnAngle){

                // Project ego and agent into corresponding future time
                Pose potentialNextPose = getNextPose(currentPose, nextAngle, dt * nextVelocity);
                float totalTime = currentNode.getTime() + dt;
                std::vector<Agent> predictedAgents = projectAgents(agents, totalTime);
                Agent futureEgo = projectEgo(potentialNextPose);

                // Check for collisions
                bool futureCollision = checkCollisions(futureEgo, predictedAgents, obstacles);
                if (!futureCollision){

                    // Add node to heap if there aren't any collisions
                    float nextNodeDistance = 1;//getPoseDistance(potentialNextPose) - getEgoPoseDistance(potentialNextPose) / 2 + noise();
                    //std::cout << "NEW DISTANCE: " << nextNodeDistance << std::endl;

                    // Set neighbors + previous 
                    Node nextNode(potentialNextPose, totalTime, nextNodeDistance);
                    currentNode.neighbors_.push_back(nextNode);
                    nextNode.setPrev(currentNode);
                    nodeHeap.push_back(nextNode);
                    nodeList.push_back(nextNode);
                    push_heap(nodeHeap.begin(), nodeHeap.end());
                    freeNodes = true;
                }
            }
        }
        
        if (!freeNodes){
            std::cout << "No free nodes" << std::endl;
        }
        
        // Increment tree iter count
        currentTreeIter++;

        // Pop from heap
        pop_heap(nodeHeap.begin(), nodeHeap.end());
        nodeHeap.pop_back();
        
        
    }
    return nodeList;
}

double generate_random_double(double min, double max) {
    double random_double = static_cast<double>(rand()) / RAND_MAX;
    //std::cout << random_double << std::endl;
    double range = max - min;
    return min + (random_double * range);
}


Node genRandomNode(double xmin, double ymin, double xmax, double ymax){
    double x = generate_random_double(xmin, xmax);
    double y = generate_random_double(ymin, ymax);
    return Node(Pose(x, y, 0), 0, 0);
}

bool checkCollision(Node currentNode, Agent a){
    float node_x = currentNode.getPose().getX();
    float node_y = currentNode.getPose().getY();
    float agent_x = a.getPose().getX();
    float agent_y = a.getPose().getY();
    if (node_x < agent_x + a.getWidth() / 2 
    && node_x > agent_x - a.getWidth() / 2
    && node_y < agent_y + a.getLength() / 2
    && node_y > agent_y - a.getLength() / 2){
        return true;
    }else{
        return false;
    }
}

bool checkCollision(Node currentNode, Obstacle o){
    float node_x = currentNode.getPose().getX();
    float node_y = currentNode.getPose().getY();
    float ob_x = o.getPose().getX();
    float ob_y = o.getPose().getY();
    if (node_x < ob_x + o.getWidth() / 2 
    && node_x > ob_x - o.getWidth() / 2
    && node_y < ob_y + o.getLength() / 2
    && node_y > ob_y - o.getLength() / 2){
        return true;
    }else{
        return false;
    }
}

bool checkCollisions(Node currentNode, std::vector<Agent> agents, std::vector<Obstacle> obstacles){
    for (int i = 0; i < agents.size(); i++){
        if (checkCollision(currentNode, agents[i])){
            return true;
        }
    }
    for (int i = 0; i < obstacles.size(); i++){
        if (checkCollision(currentNode, obstacles[i])){
            return true;
        }
    }
    return false;
}

int findLowestDistanceNodeIndex(Node currentNode, std::vector<Node> nodeList){
    float minDist = 999999;
    int minDistNodeIndex = -1;
    for (int i = 0; i < nodeList.size(); i++){
        float currentDistance = distance(currentNode, nodeList[i]);
        if (currentDistance < minDist){
            minDist = currentDistance;
            minDistNodeIndex = i;
        }
    }
    return minDistNodeIndex;
}


std::vector<Node> Vehicle::randomTreeExplore(std::vector<Agent> agents, std::vector<Obstacle> obstacles){
    
    // Set parameters
    int currentTreeIter = 0;
    int maxTreeSize = 5000;

    // Setup
    Pose currentPose = getPose();
    std::vector<Node> nodeList;
    // Node a(Pose(5, 5, 0), 0, 0);
    // Node b(Pose(2, 2, 0), 0, 0);
    // Node c(Pose(7, 1, 0), 0, 0);
    // Node d(Pose(1, 7, 0), 0, 0);
    // a.setPrev(b);
    // b.setPrev(c);
    // c.setPrev(d);
    // a.has_prev_ = true;
    // b.has_prev_ = true;
    // c.has_prev_ = true;
    // nodeList.push_back(a);
    // nodeList.push_back(b);
    // nodeList.push_back(c);
    // nodeList.push_back(d);
    Node currentNode(currentPose, 0, 0);
    nodeList.push_back(currentNode);

    for (int treeIter = 0; treeIter < maxTreeSize; treeIter++){
        Node randomNode = genRandomNode(-15.0, -15.0, 15.0, 15.0);
        if (!checkCollisions(randomNode, agents, obstacles)){
            int closestNodeIndex = findLowestDistanceNodeIndex(randomNode, nodeList);
            randomNode.setPrev(nodeList[closestNodeIndex]);
            randomNode.has_prev_ = true;
            randomNode.prev_x_ = nodeList[closestNodeIndex].getPose().getX();
            randomNode.prev_y_ = nodeList[closestNodeIndex].getPose().getY();
            nodeList.push_back(randomNode);
        }else{
            treeIter--;
        }


        // double x1 = randomNode.getPrev()->getPose().getX();
        // double y1 = randomNode.getPrev()->getPose().getY();
        // std::cout << x1 << " " << y1 << std::endl;
    }

    // for (int i = 0; i < nodeList.size(); i++){
    //     double x2 = nodeList[i].getPrev()->getPose().getX();
    //     double y2 = nodeList[i].getPrev()->getPose().getY();
    //     std::cout << x2 << " " << y2 << std::endl;
    // }
    //expect: 
    // 2, 2
    // 7, 1
    // 1, 7
    //setNodeGraph(nodeList);
    return nodeList;
}


// std::vector<Node> Vehicle::randomTreeExplore(std::vector<Agent> agents, std::vector<Obstacle> obstacles){
    
//     // Set parameters
//     int currentTreeIter = 0;
//     int maxTreeSize = 5000;
//     float dt = 0.5;
//     float minVelocity = 1; 
//     float maxVelocity = 5;
//     float velocityIncrement = 1;
//     float maxTurnAngle = M_PI / 4;
//     float minTurnAngle = M_PI / 32;
    
//     // First iter
//     Pose currentPose = getPose();
//     std::vector<Node> nodeList;
//     Node currentNode(currentPose, 0, 0);
//     nodeList.push_back(currentNode);

//     while (currentTreeIter <= maxTreeSize){
//         if (currentTreeIter == maxTreeSize){
//             currentNode = generateGoalNode();
//         }else{
//             currentNode = pickRandomNode(nodeList);
//         }
        
//         Pose newPose = pickRandomPose(currentNode.getPose(), maxTurnAngle, maxVelocity, dt);
//         Node newNode(newPose, 0, currentNode.getDistance() + distance(currentNode.getPose(), newPose));
//         Node lowestCostPrevNode = findLowestCostNeighbor(nodeList, newNode, currentNode, maxVelocity * dt);
//         newNode.prev_ = &lowestCostPrevNode;
//         nodeList.push_back(newNode);
//         if (nodeList.size() % 1000 == 0) std::cout << nodeList.size() << std::endl;
//         currentTreeIter++;
//     }

//     return nodeList;
// }

// Trajectory Vehicle::calculateTrajectory(std::vector<Agent> agents, std::vector<Obstacle> obstacles){

//     return Trajectory();
// }

Trajectory Vehicle::calculateTrajectory(std::vector<Agent> agents, std::vector<Obstacle> obstacles){

    Pose currentPose = getPose();
    float currentHeading = currentPose.getHeading();
    int numPoints = 50;
    float dt = 0.5;
    float velocity = 2.0;
    std::vector<Pose> poseList;
    for (int i = 0; i < numPoints; i++){
        float currentTime = dt * i;
        float newX = currentPose.getX() + cos(currentHeading) * currentTime * velocity;
        float newY = currentPose.getY() + sin(currentHeading) * currentTime * velocity;
        Pose newPose(newX, newY, currentHeading);
        poseList.push_back(newPose);
    }
    return Trajectory(poseList); 
}