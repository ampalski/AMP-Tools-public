#pragma once

#include "hw/HW7.h"
#include "Utils.h"

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT(int n, double p, double r, double eps) : 
            _numNodes(n), _stepSize(r), _goalChance(p), _epsilon2(eps*eps) {}

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        amp::Graph<double> getTree();
        std::map<amp::Node, Eigen::Vector2d> getMap();
    private:
        int _numNodes;
        double _stepSize;
        double _goalChance;
        double _epsilon2;
        std::map<amp::Node, Eigen::Vector2d> _samples;
        amp::Graph<double> _tree;

        amp::Node findClosestNode(Eigen::Vector2d q_rand);
};
