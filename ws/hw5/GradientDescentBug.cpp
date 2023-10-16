#include "GradientDescentBug.h"
#include <queue>

GradientDescentBug::GradientDescentBug() {
    m_qStar = 1.0;
    m_dStar = 1.0;
    m_attGain = 1.0;
    m_repGain = 1.0;
    m_wavGain = 1.0;
}

GradientDescentBug::GradientDescentBug(double dStar, double qStar, double attGain, double repGain) {
    m_qStar = qStar;
    m_dStar = dStar;
    m_attGain = attGain;
    m_repGain = repGain;
    m_wavGain = 1.0;
}

amp::Path2D GradientDescentBug::plan(const amp::Problem2D& problem) {
    // Init a path, add q_init
    // Take steps until maxSteps or within epsilon of q_goal
    // Add q_goal and return

    useWavefront = false;
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    double minDistToGo = 99999999.9;
    double dist = (path.waypoints.back() - problem.q_goal).norm();
    int ktr = 0;

    while (dist > epsilon) {
        //DEBUG("Running Step" << path.waypoints.size());
        path.waypoints.push_back(step(path.waypoints, problem));
        
        //PRINT_VEC2("Adding point: ", path.waypoints.back());

        // Find when progress gets stalled
        dist = (path.waypoints.back() - problem.q_goal).norm();
        if (!useWavefront && dist < minDistToGo) {
            minDistToGo = dist;
            ktr = 0;
            //DEBUG("MinDist reset " << dist);
        } else {
            ktr++;
        }

        // Calculate wavefront if progress is stalled
        if (!useWavefront && ktr > 20) {
            //DEBUG("Adding Wavefront");
            useWavefront = true;
            double xRange = problem.x_max - problem.x_min;
            double yRange = problem.y_max - problem.y_min;
            double xStep = xRange / (xCells - 1);
            double yStep = yRange / (yCells - 1);
            m_qStar = 0.25*std::min(xStep,yStep);
            calcWavefront(problem);
        }

        if (path.waypoints.size() > maxSteps) {
            //DEBUG("Max Steps Reached");
            break;
        }
    }

    path.waypoints.push_back(problem.q_goal);

    return path;
}

Eigen::Vector2d GradientDescentBug::step(std::vector<Eigen::Vector2d> path, const amp::Problem2D& problem) {
    Eigen::Vector2d gradU(0.0, 0.0);
    // Build up attractor 
    // Check for current distance to goal versus dStar
    // Build appropriate grad(U) terms based on dStar
    // If wavefront is needed, call the function to get the term and add it
    Eigen::Vector2d curPos = path.back();
    Eigen::Vector2d diff = curPos - problem.q_goal;
    double dist = diff.norm();

    if (useWavefront) {
        double xRange = problem.x_max - problem.x_min;
        double yRange = problem.y_max - problem.y_min;
        double xStep = xRange / (xCells - 1);
        double yStep = yRange / (yCells - 1);
        gradU += m_wavGain * std::min(xStep, yStep) * wavefrontGradient(path, problem);
        //gradU += m_wavGain * wavefrontGradient(path, problem);
    } else {
        if (dist > m_dStar) {
            gradU += (m_dStar / dist * m_attGain * diff);
        } else {
            gradU += m_attGain * diff;
        }
    }
    
    // Build up repulsor 
    // For each obstacle, for each line segment, check for closest point
    // Check against qStar
    // Build appropriate grad(U) terms based on qStar
    

    Eigen::Vector2d closestPoint;
    Eigen::Vector2d closestPointTemp;
    std::vector<Eigen::Vector2d> vertices;
    std::vector<Eigen::Vector2d> segment;
    for (amp::Obstacle2D obstacle : problem.obstacles) {
        vertices.clear();
        vertices = obstacle.verticesCCW();
        dist = 999999999.9;
        for (int i = 0; i < vertices.size(); i++) {
            segment.clear();
            segment.push_back(vertices[i]);
            int j = (i == vertices.size() - 1) ? 0 : i + 1;
            segment.push_back(vertices[j]);
            closestPointTemp = Utils::pointLineSegmentClosest(curPos, segment);
            if ((closestPointTemp - curPos).norm() < dist) {
                closestPoint = closestPointTemp; //check that this isn't by ref
                dist = (closestPoint - curPos).norm();
            }
        }

        if (dist > m_qStar) continue; // no contribution if too far away
        diff = curPos - closestPoint;
        gradU += m_repGain * (1/m_qStar - 1/dist) * (1/pow(dist,3)) * diff;
    }

    // Do collision checking based on full gradient descent step
    // if collision, line search back until step is possible
    // If no collision, test if step size is greater than distance to goal
    // reduce step size to goal if that's the case

    if (gradU(0) == 0.0 && gradU(1) == 0.0) {
        gradU(0) = amp::RNG::randd(-1.0, 1.0);
        gradU(1) = amp::RNG::randd(-1.0, 1.0);
    }
    double alpha = 1.0;
    Eigen::Vector2d newPos;

    newPos = curPos - alpha * gradU;
    while (Utils::checkStep(curPos, newPos, problem)) {
        alpha *= .7;
        newPos = curPos - alpha * gradU;
        if (alpha < .00000001) {
            return curPos;
        }
    }

    //make sure it doesn't overshoot the goal
    double stepSize = alpha * gradU.norm();
    dist = diff.norm();
    if (stepSize > dist) {
        alpha = dist / gradU.norm();
        newPos = curPos - alpha * gradU;
    }

    return newPos;

}

void GradientDescentBug::calcWavefront(const amp::Problem2D& problem) {
    // Initialize grid
    double xRange = problem.x_max - problem.x_min;
    double yRange = problem.y_max - problem.y_min;
    double xStep = xRange / (xCells - 1);
    double yStep = yRange / (yCells - 1);

    wavefront = Eigen::MatrixXi::Zero(xCells, yCells);

    // Find grid point of q_goal
    int xind = std::round((problem.q_goal(0) - problem.x_min) / xStep);
    int yind = std::round((problem.q_goal(1) - problem.y_min) / yStep);

    xind = std::clamp(xind, 0, xCells);
    yind = std::clamp(yind, 0, yCells);

    //DEBUG("qGoal is at ("<<xind<<", "<<yind<<") = "
        //<< problem.x_min+xind*xStep << ", " << problem.y_min+yind*yStep<<")");

    // Initialize wavefront
    wavefront(xind, yind) = 2;
    std::queue<std::pair<int, int>> nextNeighbors;
    nextNeighbors.emplace(xind+1, yind);
    nextNeighbors.emplace(xind-1, yind);
    nextNeighbors.emplace(xind, yind+1);
    nextNeighbors.emplace(xind, yind-1);

    // Process until complete
    int lowestNeighbor, val, ktr = 0;
    std::vector<std::pair<int, int>> zeroNeighbors;
    std::vector<std::pair<int, int>> prevNeighbors;
    std::vector<std::pair<int, int>> tempNeighbors;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d stop = Eigen::Vector2d::Zero();

    while (nextNeighbors.size() > 0) {
        std::pair<int,int> curInd = nextNeighbors.front();
        
        if (wavefront(curInd.first, curInd.second) > 0) {
            nextNeighbors.pop();
            continue;
        }
        //DEBUG("Current ind is ("<<curInd.first<<", "<<curInd.second<<") = ("
            //<< problem.x_min+curInd.first*xStep << ", " << problem.y_min+curInd.second*yStep<<")");

        ktr++;
        if (ktr%100 == 0) {
            //DEBUG(ktr);
        }
        // Check each neighbor to see if it's been visited before or not
        tempNeighbors.clear();
        zeroNeighbors.clear();
        prevNeighbors.clear();
        tempNeighbors.emplace_back(curInd.first+1, curInd.second);
        tempNeighbors.emplace_back(curInd.first-1, curInd.second);
        tempNeighbors.emplace_back(curInd.first, curInd.second+1);
        tempNeighbors.emplace_back(curInd.first, curInd.second-1);

        for (std::pair<int,int> check : tempNeighbors) {
            if (check.first < 0 || check.first > xCells-1) continue;
            if (check.second < 0 || check.second > yCells-1) continue;
            
            val = wavefront(check.first,check.second);
            //DEBUG("Checking ("<<check.first<<", "<<check.second<<") = " << val);
            if (val == 0) {
                //DEBUG("Assigned to zeroNeighbors");
                zeroNeighbors.push_back(check);
                continue;
            } else if (val == 1) {
                //DEBUG("Obstacle");
                continue; // obstacle
            } else if (val == -1) {
                //DEBUG("Already in the queue");
                continue;
            }
            //DEBUG("Assigned to prevNeighbors");
            prevNeighbors.push_back(check);
        }
        
        // Set new value
        lowestNeighbor = INT32_MAX;
        if (prevNeighbors.size() == 0) {
            //DEBUG("Accessed empty neighborhood by accident at " << ktr);
        }
        for (std::pair<int,int> check : prevNeighbors) {
            val = wavefront(check.first,check.second);
            
            if (val < lowestNeighbor) {
                lowestNeighbor = val;
            }
        }
        //DEBUG("Assigning ("<<curInd.first<<", "<<curInd.second<<") = " << lowestNeighbor+1);
        wavefront(curInd.first, curInd.second) = lowestNeighbor + 1;

        // Expand the wavefront
        start(0) = problem.x_min + curInd.first * xStep; 
        start(1) = problem.y_min + curInd.second * yStep;
        for (std::pair<int,int> check : zeroNeighbors) {
            //DEBUG("Adding ("<<check.first<<", "<<check.second<<")");

            stop(0) = problem.x_min + check.first * xStep; 
            stop(1) = problem.y_min + check.second * yStep;

            if (Utils::checkStep(start, stop, problem)) {
                wavefront(check.first, check.second) = 1;
                //DEBUG("Obstacle found");
                continue;
            }
            wavefront(check.first, check.second) = -1; // already accounted for
            nextNeighbors.push(check);
        }

        nextNeighbors.pop();
    }

}

Eigen::Vector2d GradientDescentBug::wavefrontGradient(std::vector<Eigen::Vector2d> path, const amp::Problem2D& problem) {
    // Find grid point of curPos
    Eigen::Vector2d curPos = path.back();
    double xRange = problem.x_max - problem.x_min;
    double yRange = problem.y_max - problem.y_min;
    double xStep = xRange / (xCells - 1);
    double yStep = yRange / (yCells - 1);
    
    int xind = std::round((curPos(0) - problem.x_min) / xStep);
    int yind = std::round((curPos(1) - problem.y_min) / yStep);

    xind = std::clamp(xind, 0, xCells-1);
    yind = std::clamp(yind, 0, yCells-1);

    int curValue = wavefront(xind, yind);
    //Get neighbor values
    int right = wavefront(std::clamp(xind + 1, 0, xCells-1), yind);
    int left = wavefront(std::clamp(xind - 1, 0, xCells-1), yind);
    int up = wavefront(xind, std::clamp(yind + 1, 0, yCells-1));
    int down = wavefront(xind, std::clamp(yind - 1, 0, yCells-1));

    //DEBUG("("<<xind<<", "<<yind<<") = "<<curValue);
    //DEBUG("Neighbors are " << right << " " << left<<" "<<up<<" "<<down);

    // Handle off nominal
    if (curValue == 0 || curValue == 1) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    // Search adjacent grid points for lowest neighbor
    Eigen::Vector2d waveGrad;
    int lowVal = right == 1 ? INT_MAX : right;
    waveGrad << -1, 0;

    if (left != 1) {
        if (left < lowVal || (left == lowVal && amp::RNG::randf()>0.5)) {
            lowVal = left;
            waveGrad(0) = 1.0;
            waveGrad(1) = 0.0;
        }
    }
    if (up != 1) {
        if (up < lowVal || (up == lowVal && amp::RNG::randf()>0.5)) {
            lowVal = up;
            waveGrad(0) = 0.0;
            waveGrad(1) = -1.0;
        }
    }
    if (down != 1) {
        if (down < lowVal || (down == lowVal && amp::RNG::randf()>0.5)) {
            lowVal = down;
            waveGrad(0) = 0.0;
            waveGrad(1) = 1.0;
        }
    }
    //PRINT_VEC2("Wavefront gradient is ", waveGrad);
    return waveGrad;
}
