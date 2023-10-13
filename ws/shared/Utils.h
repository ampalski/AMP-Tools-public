#pragma once

#include <Eigen/Core>
#include "AMPCore.h"
#include "hw/HW2.h"
#include "tools/Environment.h"
#include <math.h>

namespace Utils {
    bool checkStep(Eigen::Vector2d start, Eigen::Vector2d stop, const amp::Environment2D& env);

    bool checkLineSegmentIntersect(Eigen::Vector2d start, Eigen::Vector2d stop,
        Eigen::Vector2d obsStart, Eigen::Vector2d obsStop);

    bool checkCollinearOverlap(double x1, double y1, double x2, double y2, 
        double x3, double y3);

    Eigen::Vector2d rotateVec(Eigen::Vector2d vector, double angle);

    //Turn convex polygon obstacle and robot into a C-space obstacle, translate only robot case
    amp::Polygon CSObstConvPolySingle(amp::Polygon& obstacle, amp::Polygon& robot, double angle);

    //Turn convex polygon obstacle and robot into a C-space obstacle, robot can rotate
    std::vector<amp::Polygon> CSObstConvPolyRotate(amp::Polygon obstacle, amp::Polygon robot, std::vector<double> rotAngles);

    double angleOfSegment(Eigen::Vector2d segment);

    std::vector<Eigen::Vector2d> negateRotateReorder(std::vector<Eigen::Vector2d> vertices, double angle);

    Eigen::Vector2d pointLineSegmentClosest(Eigen::Vector2d point, std::vector<Eigen::Vector2d> lineSegment);
}