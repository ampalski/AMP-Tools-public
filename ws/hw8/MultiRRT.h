#pragma once

#include "hw/HW8.h"
#include "Utils.h"
#include "RRT.h"

class MyCentralMultiRRT : public amp::CentralizedMultiAgentRRT {
    public:
        MyCentralMultiRRT(int n, double p, double r, double eps) : 
            _numNodes(n), _stepSize(r), _goalChance(p), _epsilon2(eps*eps) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        amp::Graph<double> getTree();
        std::map<amp::Node, Eigen::VectorXd> getMap();
    private:
        int _numNodes;
        double _stepSize;
        double _goalChance;
        double _epsilon2;
        int _numRobots;
        std::map<amp::Node, Eigen::VectorXd> _samples;
        amp::Graph<double> _tree;

        amp::Node findClosestNode(Eigen::VectorXd q_rand);
};

class MyDecentralMultiRRT : public amp::DecentralizedMultiAgentRRT {
    public:
        MyDecentralMultiRRT(int n, double p, double r, double eps) : 
            _numNodes(n), _stepSize(r), _goalChance(p), _epsilon2(eps*eps) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        amp::Graph<double> getTree();
        std::map<amp::Node, Eigen::Vector2d> getMap();
        std::vector<amp::Path2D> getPaths();
    private:
        int _numNodes;
        double _stepSize;
        double _goalChance;
        double _epsilon2;
        int _numRobots;
        std::map<amp::Node, Eigen::Vector2d> _samples;
        std::map<amp::Node, int> _steps;
        amp::Graph<double> _tree;
        std::vector<amp::Path2D> _paths;

        amp::Node findClosestNode(Eigen::Vector2d q_rand, amp::Node base);
        std::pair<Eigen::VectorXd, Eigen::VectorXd> buildMultiStates(amp::Node q_near, Eigen::Vector2d q_goal);
};