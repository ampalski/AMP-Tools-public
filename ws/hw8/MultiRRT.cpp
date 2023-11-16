#include "MultiRRT.h"

#include <map>
#include <list> 
#include <iterator>

amp::Node MyCentralMultiRRT::findClosestNode(Eigen::VectorXd q_rand) {
    
    double temp;
    amp::Node closest = 0;
    Eigen::VectorXd diff = q_rand - _samples[0];
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

amp::Graph<double> MyCentralMultiRRT::getTree() {
    return _tree;
}

std::map<amp::Node, Eigen::VectorXd> MyCentralMultiRRT::getMap() {
    return _samples;
}

amp::MultiAgentPath2D MyCentralMultiRRT::plan(const amp::MultiAgentProblem2D& problem) {
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
    _tree.clear();

    int m = problem.numAgents();
    _numRobots = m;
    int ind0, ind1, ktr;
    Eigen::VectorXd scratch(2*m);
    Eigen::Vector2d scratch2d;

    amp::Node curNode = 0, q_near;
    for (int i = 0; i < m; i++) {
        ind0 = 2*i;
        ind1 = 2*i+1;
        scratch2d = problem.agent_properties.at(i).q_init;
        scratch(ind0) = scratch2d(0);
        scratch(ind1) = scratch2d(1);
    }
    _samples[curNode++] = scratch;
    Eigen::VectorXd q_rand, q_new, dir, diff;
    Eigen::VectorXd useStepSize(2*m);
    double distToGo, scratchd;
    bool solnFoundAgents[m] = {0};
    //DEBUG("Init complete");
    // Main loop
    while (!solnFound) {
        // Generate random sample
        //DEBUG("Top of loop");
        if (amp::RNG::randd() < _goalChance) {
            for (int i = 0; i < m; i++) {
                ind0 = 2*i;
                ind1 = 2*i+1;
                scratch2d = problem.agent_properties.at(i).q_goal;
                scratch(ind0) = scratch2d(0);
                scratch(ind1) = scratch2d(1);
            }
            q_rand = scratch;
        } else {
            q_rand = Utils::generateCollisionFreeSample(problem, m);
            if (q_rand.rows() == m)  {
                //DEBUG("Invalid random point, continuing");
                for (int i = 0; i < m; i++) {
                    ind0 = 2*i;
                    ind1 = 2*i+1;
                    scratch2d = problem.agent_properties.at(i).q_goal;
                    scratch(ind0) = scratch2d(0);
                    scratch(ind1) = scratch2d(1);
                }
                q_rand = scratch;
            }
        }
        //DEBUG("Random point found");
        //PRINT_VEC2("Random point: ", q_rand);

        // Find Closest Node
        q_near = findClosestNode(q_rand);
        //DEBUG("Closest Node is " << q_near);

        // Generate path 
        dir = q_rand - _samples[q_near];
        for (int i = 0; i < m; i++) {
            ind0 = 2*i;
            ind1 = 2*i+1;
            Eigen::Vector2i inds; inds << ind0, ind1;
            scratch2d = dir(inds).normalized();
            dir(ind0) = scratch2d(0);
            dir(ind1) = scratch2d(1);
        }
        //DEBUG("Path generated");

        // Step towards q_rand, adding to the tree as it goes
        stepDone = false;
        ktr = 0;
        while (!stepDone) {
            stepDone = true;
            ktr++;
            for (int i = 0; i < m; i++) {
                ind0 = 2*i;
                ind1 = 2*i+1;
                Eigen::Vector2i inds; inds << ind0, ind1;
                distToGo = (_samples[q_near](inds) - q_rand(inds)).norm();
                /*if (solnFoundAgents[i]) {
                    useStepSize(ind0) = 0;
                    useStepSize(ind1) = 0;
                } else */
                if (distToGo > _stepSize) {
                    useStepSize(ind0) = _stepSize;
                    useStepSize(ind1) = _stepSize;
                    stepDone = false;
                } else {
                    useStepSize(ind0) = distToGo;
                    useStepSize(ind1) = distToGo;
                }
            }
            //DEBUG("Found step size");
            //DEBUG("Q_new: "<< _samples[q_near].rows() <<" + " << useStepSize.rows() << " * " << dir.rows());

            q_new = _samples[q_near] + useStepSize.cwiseProduct(dir);

            if (Utils::isMultiAgentCollisionStep(problem, _samples[q_near], q_new, m)) {
                stepDone = true;
                continue;
            } 
            //PRINT_VEC2("Adding: ", q_new);
            _samples[curNode] = q_new;
            //DEBUG("Connecting " << q_near<<" and "<<curNode);
            _tree.connect(q_near, curNode, (_samples[curNode] - _samples[q_near]).norm());
            //DEBUG("Success");
            q_near = curNode;
            curNode++;
            
            // Check for goal
            solnFound = true;
            for (int i = 0; i < m; i++) {
                /*if (solnFoundAgents[i]) {
                    continue;
                }*/
                ind0 = 2*i;
                ind1 = 2*i+1;
                Eigen::Vector2i inds; inds << ind0, ind1;
                diff = q_new(inds) - problem.agent_properties.at(i).q_goal;
                if (diff.dot(diff) < _epsilon2) {
                    solnFoundAgents[i] = true;
                } else {
                    solnFound = false;
                }
            }
            
            if (solnFound) {
                //DEBUG("Solution found " << q_near);
                goalNode = q_near;
                break;
            }
            if (ktr > 1000) {
                //DEBUG("Too many steps towards random point, break");
                break;
            }
            //DEBUG("Success");
        }

        // Break if too many loops
        if (curNode > _numNodes) {
            //DEBUG("Too many nodes, no soln found");
            break;
        }
    }

    amp::MultiAgentPath2D path;
    // Fail if solution not found
    if (!solnFound) {
        path.valid = false;
        for (int k = 0; k < m; k++) {
            amp::Path2D iPath;
            iPath.valid = false;
            iPath.waypoints.clear();
            iPath.waypoints.push_back(problem.agent_properties.at(k).q_init);
            iPath.waypoints.push_back(problem.agent_properties.at(k).q_goal);
            path.agent_paths.push_back(iPath);
        }
        return path;
    }

    // Convert to Path2D
    //DEBUG("Converting to Path");
    // path has a std::vector<Path2D> agent_paths value that needs to be filled out
    path.valid = true;
    amp::Path2D iPath;
    iPath.valid = true;
    amp::Node tempNode;
    for (int i = 0; i < m; i++) {
        iPath.waypoints.clear();
        ind0 = 2*i;
        ind1 = 2*i+1;
        Eigen::Vector2i inds; inds << ind0, ind1;
        // the while loop below goes here
        solnFound = true;
        tempNode = goalNode;
        while (solnFound) {
            iPath.waypoints.insert(iPath.waypoints.begin(), _samples[tempNode](inds));
            //DEBUG("At Node "<< tempNode);
            if (tempNode == 0) {
                solnFound = false;
                continue;
            }
            tempNode = _tree.parents(tempNode)[0];
        }
        iPath.waypoints.push_back(problem.agent_properties.at(i).q_goal);
        //DEBUG("Pushed goal");
        path.agent_paths.push_back(iPath);
        //DEBUG("Pushed path");
    }

    return path;
    
}

amp::Graph<double> MyDecentralMultiRRT::getTree() {
    return _tree;
}

std::map<amp::Node, Eigen::Vector2d> MyDecentralMultiRRT::getMap() {
    return _samples;
}

std::vector<amp::Path2D> MyDecentralMultiRRT::getPaths() {
    return _paths;
}

amp::MultiAgentPath2D MyDecentralMultiRRT::plan(const amp::MultiAgentProblem2D& problem) {
    /*
    1) for each robot in priority order
    2) Execute single-robot GoalBiasRRT, with:
    3) Collision checking is against obstacles a disk, AND
    4) collision check against all previously planned robots
    5) whole path comes back invalid if any individual robot is invalid
    */
    
    // Init
    int m = problem.numAgents();
    _numRobots = m;
    bool solnFound, stepDone;
    std::vector<amp::Node> goalNodes;
    _samples.clear();
    _steps.clear();
    _tree.clear();
    _paths.clear();
    amp::Node curNode = 0, q_near, base;
    int iNodes = 0;
    Eigen::Vector2d q_rand, q_new, dir, diff;
    double useStepSize;
    double distToGo;
    amp::Path2D iPath;
    amp::MultiAgentPath2D path;
    path.valid = true;

    // Loop through each robot
    int mi = 0;
    for (amp::CircularAgentProperties robot : problem.agent_properties) {
        ++mi;
        base = curNode;
        _samples[curNode++] = robot.q_init;
        _steps[base] = 0;
        solnFound = false;
        stepDone = false;
        iNodes = 0;
        iPath.waypoints.clear();
        //DEBUG("Running agent " << mi);
        // GoalBiasRRT main loop
        while (!solnFound) {
            // Generate random sample
            if (amp::RNG::randd() < _goalChance) {
                q_rand = robot.q_goal;
            } else {
                q_rand = Utils::genColFreeSample(problem);
                if (q_rand(0) == problem.x_max+1) {
                    q_rand = robot.q_goal;
                }
            }
            //PRINT_VEC2("Random point: ", q_rand);

            // Find closest node
            q_near = findClosestNode(q_rand, base);
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
                auto multiStates = buildMultiStates(q_near, q_new);
                if (Utils::isMultiAgentCollisionStep(problem, multiStates.first, multiStates.second, mi)) {
                    //DEBUG("Collision found");
                    stepDone = true;
                    continue;
                }

                _samples[curNode] = q_new;
                _steps[curNode] = _steps[q_near] + 1;
                //DEBUG("Connecting " << q_near<<" and "<<curNode <<", edge length "<< useStepSize);
                _tree.connect(q_near, curNode, useStepSize);
                q_near = curNode;
                curNode++;
                iNodes++;

                // Check for goal
                diff = q_new - robot.q_goal;
                if (diff.dot(diff) < _epsilon2) {
                    solnFound = true;
                    goalNodes.push_back(q_near);
                    break;
                }
            }
            // Break if too many loops
            if (iNodes > _numNodes) {
                break;
            }
        }
        if (!solnFound) {
            path.valid = false;
            path.agent_paths = _paths;
            for (int k = mi-1; k < m; k++) {
                iPath.valid = false;
                iPath.waypoints.clear();
                iPath.waypoints.push_back(problem.agent_properties.at(k).q_init);
                iPath.waypoints.push_back(problem.agent_properties.at(k).q_goal);
                path.agent_paths.push_back(iPath);
            }
            return path;
        }
        //Convert to Path2D
        iPath.valid = true;
        q_near = goalNodes.back();
        while (solnFound) {
            iPath.waypoints.insert(iPath.waypoints.begin(), _samples[q_near]);
            if (q_near == base) {
                solnFound = false;
                continue;
            }
            q_near = _tree.parents(q_near)[0];
        }
        iPath.waypoints.push_back(robot.q_goal);

        // Add to path
        _paths.push_back(iPath);
    }
    
    path.agent_paths = _paths;

    return path;
    
}

amp::Node MyDecentralMultiRRT::findClosestNode(Eigen::Vector2d q_rand, amp::Node base) {
    double temp;
    amp::Node closest = base;
    Eigen::Vector2d diff = q_rand - _samples[closest];
    double dist2 = diff.dot(diff);
    amp::Node key;
    Eigen::Vector2d value;
    for (int i = base; i < _samples.size(); i++) {
        key = i;
        value = _samples[key];
        diff = value - q_rand;
        temp = diff.dot(diff);
        if (temp < dist2) {
            dist2 = temp;
            closest = key;
        }
    }
    return closest;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> MyDecentralMultiRRT::buildMultiStates(amp::Node q_near, Eigen::Vector2d q_goal) {
    int m = _paths.size();
    int n = 2*(m+1);
    int step = _steps[q_near];
    int useStep, ind0, ind1;
    Eigen::VectorXd start(n), end(n);

    for (int i = 0; i < m; i++) {
        ind0 = 2*i;
        ind1 = 2*i+1;
        //useStep = (step > _paths[i].waypoints.size()) ? (_paths[i].waypoints.size()-1) : step;
        useStep = std::clamp(step, 0, (int)_paths[i].waypoints.size()-1);
        start(ind0) = _paths[i].waypoints[useStep](0);
        start(ind1) = _paths[i].waypoints[useStep](1);
        //DEBUG("Start: " << start(0) << " " << start(1)<< " " << start(2)<< " " << start(3));

        //useStep = ((step+1) > _paths[i].waypoints.size()) ? (_paths[i].waypoints.size()-1) : (step+1);
        useStep = std::clamp(step+1, 0, (int)_paths[i].waypoints.size()-1);
        end(ind0) = _paths[i].waypoints[useStep](0);
        end(ind1) = _paths[i].waypoints[useStep](1);
    }

    start(n-2) = _samples[q_near](0);
    start(n-1) = _samples[q_near](1);
    //DEBUG("Start: " << start(0) << " " << start(1)<< " " << start(2)<< " " << start(3));
    end(n-2) = q_goal(0);
    end(n-1) = q_goal(1);
    //DEBUG("End: " << end(0) << " " << end(1)<< " " << end(2)<< " " << end(3));

    return std::make_pair(start, end);

}