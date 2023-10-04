#include "Utils.h"

// Return true if the possible step intersects an obstacle
bool Utils::checkStep(Eigen::Vector2d start, Eigen::Vector2d stop, const amp::Problem2D& problem) {
    // Iterate through each obstacle, then each line segment, to check if the bug
    // motion from `start` to `stop` intersects those segments

    // Get all Obstacles
    for(amp::Obstacle2D obst : problem.obstacles){
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

amp::Polygon Utils::CSObstConvPolyTranslate(amp::Polygon& obstacle, amp::Polygon& robot) {
    // Init variables
    int oInd = 0, rInd = 0;
    std::vector<Eigen::Vector2d> oVertices = obstacle.verticesCCW();
    int n = oVertices.size();
    std::vector<Eigen::Vector2d> rVertices = negateRotateReorder(robot.verticesCCW(), 0.0);
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
        PRINT_VEC2("Rotated, negated ", vertices[i]);
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
        PRINT_VEC2("New robot ", newVertices.back());
        ind++;
        if (ind == n) {
            ind = 0;
        }
    }
    return newVertices;
}