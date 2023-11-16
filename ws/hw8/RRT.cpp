#include "RRT.h"

#include <map>
#include <list> 
#include <iterator>

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    /*
    1) Init tree with root = q_init
    while q_goal not found:
    2) generate random sample
        a) if rand() < p, q_rand = q_goal
        b) else, collisionfreesample
    3) find closest node to q_rand, designate q_near
    4) generate path from q_near to q_rand
    5) take step along that path according to step size r
        a) if collision free, add as q_new
        b) continue stepping until q_rand is reached or collision is found
    6) add all collision free paths to tree
    7) If (q_new-q_goal).norm() < epsilon, consider goal reached and return
    */
    
    // Init
    bool solnFound = false;
    bool stepDone = false;
    amp::Node goalNode;
    _samples.clear();
    //DEBUG("Tree Size: " << _tree.nodes().size());
    _tree.clear();
    //DEBUG("Tree Size: " << _tree.nodes().size());

    amp::Node curNode = 0, q_near;
    _samples[curNode++] = problem.q_init;
    Eigen::Vector2d q_rand, q_new, dir, diff;
    double useStepSize;
    double distToGo;

    // Main loop
    while (!solnFound) {
        // Generate random sample
        if (amp::RNG::randd() < _goalChance) {
            q_rand = problem.q_goal;
        } else {
            q_rand = Utils::generateCollisionFreeSample(problem);
        }

        //PRINT_VEC2("Random point: ", q_rand);

        // Find Closest Node
        q_near = findClosestNode(q_rand);
        //PRINT_VEC2("Closest Node is ", _samples[q_near]);

        // Generate path 
        dir = (q_rand - _samples[q_near]).normalized();

        // Step towards q_rand, adding to the tree as it goes
        stepDone = false;
        while (!stepDone) {
            distToGo = (_samples[q_near] - q_rand).norm();
            if (distToGo > _stepSize) {
                useStepSize = _stepSize;
            } else {
                useStepSize = distToGo;
                stepDone = true;
            }
            q_new = _samples[q_near] + useStepSize * dir;
            if (Utils::checkStep(_samples[q_near], q_new, problem)) {
                stepDone = true;
                continue;
            } 
            //PRINT_VEC2("Adding: ", q_new);
            _samples[curNode] = q_new;
            //DEBUG("Connecting " << q_near<<" and "<<curNode <<", edge length "<< useStepSize);
            _tree.connect(q_near, curNode, useStepSize);
            //DEBUG("Success");
            q_near = curNode;
            curNode++;
            
            // Check for goal
            diff = q_new - problem.q_goal;
            if (diff.dot(diff) < _epsilon2) {
                solnFound = true;
                goalNode = q_near;
                break;
            }
        }

        // Break if too many loops
        if (curNode > _numNodes) {
            break;
        }
    }

    amp::Path2D path;
    // Fail if solution not found
    if (!solnFound) {
        path.valid = false;
        return path;
    }

    // Convert to Path2D
    //DEBUG("Converting to Path");
    path.valid = true;
    while (solnFound) {
        path.waypoints.insert(path.waypoints.begin(), _samples[goalNode]);
        if (goalNode == 0) {
            solnFound = false;
            continue;
        }
        goalNode = _tree.parents(goalNode)[0];
    }
    path.waypoints.push_back(problem.q_goal);
    return path;
}

amp::Node MyRRT::findClosestNode(Eigen::Vector2d q_rand) {
    
    double temp;
    amp::Node closest = 0;
    Eigen::Vector2d diff = q_rand - _samples[0];
    double dist2 = diff.dot(diff);

    for (const auto& [key, value] : _samples) {
        diff = value - q_rand;
        temp = diff.dot(diff);
        if (temp < dist2) {
            dist2 = temp;
            closest = key;
        }
    }
    return closest;
}

amp::Graph<double> MyRRT::getTree() {
    return _tree;
}

std::map<amp::Node, Eigen::Vector2d> MyRRT::getMap() {
    return _samples;
}