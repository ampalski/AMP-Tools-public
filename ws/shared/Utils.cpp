#include "Utils.h"
#include <queue>

// Return true if the possible step intersects an obstacle
bool Utils::checkStep(Eigen::Vector2d start, Eigen::Vector2d stop, const amp::Environment2D& env) {
    // Iterate through each obstacle, then each line segment, to check if the bug
    // motion from `start` to `stop` intersects those segments

    // Get all Obstacles
    for(amp::Obstacle2D obst : env.obstacles){
        //Get all vertices
        std::vector<Eigen::Vector2d>& vertices = obst.verticesCCW();

        for(int i = 0; i < vertices.size(); i++){
            int j = (i == vertices.size() - 1) ? 0 : i + 1;

            if (Utils::checkLineSegmentIntersect(start, stop, vertices[i], vertices[j])) {
                //DEBUG("Found Hit");
                //PRINT_VEC2("V1", vertices[i]);
                //PRINT_VEC2("V2", vertices[j]);
                return true;
            }
        }
    }

    return false; 
}

bool Utils::checkLineSegmentIntersect(Eigen::Vector2d start, Eigen::Vector2d stop,
        Eigen::Vector2d obsStart, Eigen::Vector2d obsStop) {

    //if parallel, check for colinear
    //if no, return false
    //if yes, check if end points are within the other line segment

    double denom = (start(0) - stop(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - stop(1)) * (obsStart(0) - obsStop(0));

    //DEBUG(denom);
    if (denom == 0) {
        //Parallel, check for collinear and overlap
        //just in case, check all 4 cases
        return checkCollinearOverlap(start(0), start(1), stop(0), stop(1), obsStart(0), obsStart(1)) ||
            checkCollinearOverlap(start(0), start(1), stop(0), stop(1), obsStop(0), obsStop(1)) ||
            checkCollinearOverlap(obsStart(0), obsStart(1), obsStop(0), obsStop(1), start(0), start(1)) ||
            checkCollinearOverlap(obsStart(0), obsStart(1), obsStop(0), obsStop(1), stop(0), stop(1));
    }

    // Basic line segment intersection check. If intersection occurs between
    // 0 and 1 for both segments, there was an intersection

    double t = (start(0) - obsStart(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - obsStart(1)) * (obsStart(0) - obsStop(0));
    t /= denom;
    //DEBUG(t);
    if (t < 0.0 || t > 1) {
        return false;
    }

    t = (start(0) - obsStart(0)) * (start(1) - stop(1)) - 
        (start(1) - obsStart(1)) * (start(0) - stop(0));
    t /= denom;
    //DEBUG(t);
    if (t < 0.0 || t > 1) {
        return false;
    }

    return true;
}

bool Utils::checkCollinearOverlap(double x1, double y1, double x2, double y2, 
        double x3, double y3) {
    //shoelace formula to check all three points are collinear
    //true if area of triangle = 0

    double area2 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);

    if (abs(area2) > .0000001) {
        //not collinear
        return false;
    }

    //If collinear, check if point 3 is bounded by points 1 and 2

    return (x3 <= std::max(x1, x2) && x3 >= std::min(x1, x2) && 
        y3 <= std::max(y1, y2) && y3 >= std::min(y1, y2));

}

bool Utils::isPointInObstacles(Eigen::Vector2d point, const amp::Environment2D& env) {
    // Loop through each obstacle in the environment, check it against the pointinpolygon fn
    // Get all Obstacles
    //PRINT_VEC2("Checking for Obstacles at ", point);
    for(amp::Obstacle2D obst : env.obstacles){
        if (isPointInPolygon(point, obst)) {
            //DEBUG("Found obstacle");
            return true;
        }
    }
    return false;
}

bool Utils::isPointInPolygon(Eigen::Vector2d point, const amp::Obstacle2D& obst) {
    /* 
    General idea is to take a line segment from the point to infinity and check
    how many times that segment intersects the polygon. If an even number (or 0), 
    return false, odd return true (in the polygon). 

    Need to check for edge cases:
    -If the intersection is a vertex, it should return 2 segments in collision.
    Fine if outside, not fine if inside.
    -If the point-infinity line is collinear with the segment, have to check 
    if the point lies on the segment.
    */
    

    // Build test segment
    Eigen::Vector2d extreme = point + Eigen::Vector2d(999999.9, 0);
    //PRINT_VEC2("From Point", point);
    //PRINT_VEC2("To", extreme);

    // Loop through each line segment
    std::vector<Eigen::Vector2d> vertices = obst.verticesCCW();
    Eigen::Vector2d obsStart, obsStop;
    double denom, t;
    int numIntersects = 0;
    for(int i = 0; i < vertices.size(); i++){
        int j = (i == vertices.size() - 1) ? 0 : i + 1;
        obsStart = vertices[i];
        obsStop = vertices[j];
        //PRINT_VEC2("From Point", obsStart);
        //PRINT_VEC2("To", obsStop);

        //if parallel, check for colinear
        //if yes, check if end points are within the other line segment

        denom = (point(0) - extreme(0)) * (obsStart(1) - obsStop(1)) - 
            (point(1) - extreme(1)) * (obsStart(0) - obsStop(0));

        if (denom == 0) {
            //Parallel, check for collinear and overlap
            if (checkCollinearOverlap(obsStart(0), obsStart(1), obsStop(0), obsStop(1), point(0), point(1))) {
                    //on the line segment, return true
                    //DEBUG("On the line");
                    return true;
            }
            //DEBUG("Colinear, no overlap");
            continue;
        }

        // Basic line segment intersection check. If intersection occurs between
        // 0 and 1 for both segments, there was an intersection

        // if intersection occurs AT 0 or 1 on the polygon segment, handle vertex

        t = (point(0) - obsStart(0)) * (obsStart(1) - obsStop(1)) - 
            (point(1) - obsStart(1)) * (obsStart(0) - obsStop(0));
        t /= denom;

        if (t < 0.0 || t > 1) {
            //DEBUG("intersection outside ray");
            continue;
        }

        t = (point(0) - obsStart(0)) * (point(1) - extreme(1)) - 
            (point(1) - obsStart(1)) * (point(0) - extreme(0));
        t /= denom;

        if (t < 0.0 || t > 1) {
            //DEBUG("intersection outside segment");
            continue;
        }

        if (t == 0 || t == 1) {
            // only keep if the other vertex is below the point-extreme line
            // will result in 0 or 2 if outside the polygon and hitting the vertex
            // at a tangent, 1 if inside the polygon and exiting through the vertex
            //DEBUG("Hit vertex");
            double minY = std::min(obsStart(1), obsStop(1));
            if (minY == point(1)) {
                //DEBUG("A bad vertex");
                continue;
            }
        }
        //DEBUG("Incrementing");
        numIntersects++;
    }
    //DEBUG("Total intersects " << numIntersects);
    return numIntersects % 2 == 1 ? true : false;
}

Eigen::Vector2d Utils::rotateVec(Eigen::Vector2d vector, double angle) {
    //angle in degrees, translate to radians
    double rad = angle * M_PI / 180;
    
    Eigen::Matrix2d R;
    double c = cos(rad);
    double s = sin(rad);
    R(0,0) = c;
    R(1,1) = c;
    R(0,1) = -s;
    R(1,0) = s;

    return R * vector; 
}

amp::Polygon Utils::CSObstConvPolySingle(amp::Polygon& obstacle, amp::Polygon& robot, double angle) {
    // Init variables
    int oInd = 0, rInd = 0;
    std::vector<Eigen::Vector2d> oVertices = obstacle.verticesCCW();
    int n = oVertices.size();
    std::vector<Eigen::Vector2d> rVertices = negateRotateReorder(robot.verticesCCW(), angle);
    int m = rVertices.size();

    std::vector<Eigen::Vector2d> csObstacle;

    // Start looping and building up the vertices for the C-Space Obstacle
    while (csObstacle.size() <= (n+m)) {
        // Handle wrapping
        int oindLow = oInd;
        int oindHi = oInd + 1;
        if (oInd == (n-1)) {
            oindHi = 0;
        } else if (oInd == n) {
            oindHi = 1;
            oindLow = 0;
        }
        double oAngle = Utils::angleOfSegment(oVertices[oindHi] - oVertices[oindLow]);
        oAngle = (oInd == n) ? oAngle + 2 * M_PI : oAngle;
        //PRINT_VEC2("first "<< oindLow,oVertices[oindLow]);
        //PRINT_VEC2("second " << oindHi,oVertices[oindHi]);

        int rindLow = rInd;
        int rindHi = rInd + 1;
        if (rInd == (m-1)) {
            rindHi = 0;
        } else if (rInd == m) {
            rindHi = 1;
            rindLow = 0;
        }
        double rAngle = Utils::angleOfSegment(rVertices[rindHi] - rVertices[rindLow]);
        rAngle = (rInd == m) ? rAngle + 2 * M_PI : rAngle;
        //PRINT_VEC2("first "<<rindLow,rVertices[rindLow]);
        //PRINT_VEC2("second " << rindHi,rVertices[rindHi]);

        //DEBUG("Angles are " << oAngle << " and " << rAngle);

        // Push to c-space vertices
        csObstacle.push_back(oVertices[oindLow] + rVertices[rindLow]);

        //check for lower angle, increment corresponding index
        if (std::abs(rAngle - oAngle) < .000001) {
            oInd++;
            rInd++;
        } else if (oAngle < rAngle) {
            oInd++;
        } else {
            rInd++;
        }
        if (oInd == n && rInd == m) {
            break;
        }
    }

    amp::Polygon csPolygon(csObstacle);

    return csPolygon;
}

double Utils::angleOfSegment(Eigen::Vector2d segment) {
    double angle = atan2(segment(1), segment(0));
    angle = (angle < 0) ? angle + 2 * M_PI : angle;
    return angle;
}

std::vector<Eigen::Vector2d> Utils::negateRotateReorder(std::vector<Eigen::Vector2d> vertices, double angle) {
    //First pass, negate and rotate each vertex
    const int n = vertices.size();

    for (int i = 0; i < n; i++) {
        vertices[i] = -Utils::rotateVec(vertices[i], angle);
        //PRINT_VEC2("Rotated, negated ", vertices[i]);
    }

    //Second pass, find lowest y-value
    
    int ind = n + 1;
    double hi = 999999999999999;
    Eigen::Vector2d offset;

    for (int i = 0; i < n; i++) {
        // Also check for left-most if multiple vertices at same y-value
        if (vertices[i](1) < hi || 
                (vertices[i](1) == hi && vertices[i](0) < vertices[ind](0))) {
            hi = vertices[i](1);
            ind = i;
            offset = vertices[i];
        }
    }

    //Construct new, re-ordered vertices
    std::vector<Eigen::Vector2d> newVertices;
    while (newVertices.size() < n) {
        //newVertices.push_back(vertices[ind] - offset);
        newVertices.push_back(vertices[ind]);
        //PRINT_VEC2("New robot ", newVertices.back());
        ind++;
        if (ind == n) {
            ind = 0;
        }
    }
    return newVertices;
}

std::vector<amp::Polygon> Utils::CSObstConvPolyRotate(amp::Polygon obstacle, amp::Polygon robot, std::vector<double> rotAngles) {
    std::vector<amp::Polygon> cSpaceObstacle;

    for (double angle : rotAngles) {
        cSpaceObstacle.push_back(CSObstConvPolySingle(obstacle, robot, angle * 180.0 / M_PI));
    }
    return cSpaceObstacle;
}

Eigen::Vector2d Utils::pointLineSegmentClosest(Eigen::Vector2d point, std::vector<Eigen::Vector2d> lineSegment) {
    /* 
    With points on line segment A and B, and separate point C, to find the
    closest point on AB to C: first project vector AC onto vector AB. If that
    projection falls between and and B, the closest point is the projection. 
    If it falls beyond A, A is the closest point. If it falls beyond B, B 
    is the closest point.
    */
    
    Eigen::Vector2d AB = lineSegment[1] - lineSegment[0];
    Eigen::Vector2d AC = point - lineSegment[0];

    double dotProduct = AB.dot(AC);
    double norm2 = AB.dot(AB);

    double projLength = dotProduct / norm2;

    //Short circuit if beyond the line segment
    if (projLength > 1) {
        return lineSegment[1];
    } else if (projLength < 0) {
        return lineSegment[0];
    }

    Eigen::Vector2d closestPoint = lineSegment[0] + projLength * AB;
    return closestPoint;
}


Eigen::MatrixXi Utils::buildWavefront(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool wrap) {
    //DEBUG("Building wavefront");
    
    // Build grid for wavefront
    // Do a first pass through grid_cspace to check for obstacles
    // Then start at q_goal, and propagate outwards

    // Initialize grid
    auto[xCells, yCells] = grid_cspace.size();
    double xRange = grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first;
    double yRange = grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first;
    double xStep = xRange / (xCells - 1);
    double yStep = yRange / (yCells - 1);

    Eigen::MatrixXi wavefront = Eigen::MatrixXi::Zero(xCells, yCells);

    // Find grid point of q_goal
    auto[xind,yind] = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));

    //DEBUG("qGoal is at ("<<xind<<", "<<yind<<") = "
        //<< problem.x_min+xind*xStep << ", " << problem.y_min+yind*yStep<<")");

    // Initialize wavefront
    // First grab obstacles
    
    for (int i = 0; i < xCells; i++) {
        for (int j = 0; j < yCells; j++) {
            if (grid_cspace(i,j)) {
                //DEBUG("Setting point " << i << " " << j);
                wavefront(i,j) = 1;
            }
        }
    }

    // Set goal = 2, initialize Neighbors queue
    //DEBUG("Initializing queue");
    wavefront(xind, yind) = 2;
    std::queue<std::pair<int, int>> nextNeighbors;
    if (wrap) {
        nextNeighbors.emplace(Utils::clampWrap((int)xind+1,0,(int)xCells-1), yind);
        nextNeighbors.emplace(Utils::clampWrap((int)xind-1,0,(int)xCells-1), yind);
        nextNeighbors.emplace(xind, Utils::clampWrap((int)yind+1,0,(int)yCells-1));
        nextNeighbors.emplace(xind, Utils::clampWrap((int)yind-1,0,(int)yCells-1));
    } else{ 
        nextNeighbors.emplace(std::clamp((int)xind+1,0,(int)xCells-1), yind);
        nextNeighbors.emplace(std::clamp((int)xind-1,0,(int)xCells-1), yind);
        nextNeighbors.emplace(xind, std::clamp((int)yind+1,0,(int)yCells-1));
        nextNeighbors.emplace(xind, std::clamp((int)yind-1,0,(int)yCells-1));
    }
    

    // Process until complete
    int lowestNeighbor, val, ktr = 0;
    std::vector<std::pair<int, int>> zeroNeighbors;
    std::vector<std::pair<int, int>> prevNeighbors;
    std::vector<std::pair<int, int>> tempNeighbors;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d stop = Eigen::Vector2d::Zero();

    while (nextNeighbors.size() > 0) {
        std::pair<int,int> curInd = nextNeighbors.front();
        //DEBUG("Checking point (" << curInd.first << ", " << curInd.second << ")");
        if (wavefront(curInd.first, curInd.second) > 0) {
            // Already handled or an obstacle
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
        if (wrap) {
            tempNeighbors.emplace_back(Utils::clampWrap(curInd.first+1,0,(int)xCells-1), curInd.second);
            tempNeighbors.emplace_back(Utils::clampWrap(curInd.first-1,0,(int)xCells-1), curInd.second);
            tempNeighbors.emplace_back(curInd.first, Utils::clampWrap(curInd.second+1,0,(int)yCells-1));
            tempNeighbors.emplace_back(curInd.first, Utils::clampWrap(curInd.second-1,0,(int)yCells-1));
        } else {
            tempNeighbors.emplace_back(std::clamp(curInd.first+1,0,(int)xCells-1), curInd.second);
            tempNeighbors.emplace_back(std::clamp(curInd.first-1,0,(int)xCells-1), curInd.second);
            tempNeighbors.emplace_back(curInd.first, std::clamp(curInd.second+1,0,(int)yCells-1));
            tempNeighbors.emplace_back(curInd.first, std::clamp(curInd.second-1,0,(int)yCells-1));
        }

        for (std::pair<int,int> check : tempNeighbors) {
            if (check.first < 0 || check.first >= xCells) continue;
            if (check.second < 0 || check.second >= yCells) continue;
            
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
        for (std::pair<int,int> check : zeroNeighbors) {
            //DEBUG("Adding ("<<check.first<<", "<<check.second<<")");
            // Should have already accounted for obstacle checking
            wavefront(check.first, check.second) = -1; // already accounted for
            nextNeighbors.push(check);
        }

        nextNeighbors.pop();
    }

    return wavefront;
}

int Utils::clampWrap(int val, int lo, int hi) {
    if (val >= lo && val <= hi) {
        return val;
    } else if (val < lo) {
        int diff = lo - val - 1;
        return hi - diff;
    } else {
        int diff = val - hi - 1;
        return lo + diff;
    }
}