#pragma once

#include "hw/HW7.h"
#include "Utils.h"
#include "AStar.h"

struct PrmHeuristic : public amp::SearchHeuristic {
    virtual double operator()(amp::Node node) const override;

    std::map<amp::Node, Eigen::Vector2d> nodes;
    Eigen::Vector2d q_goal;
};

class MyPRM : public amp::PRM2D {
    public:
        MyPRM(int n, double r, bool smooth) : _numNodes(n), 
            _connectRadius(r), _smoothing(smooth) {}

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        amp::Graph<double> getGraph();
        std::map<amp::Node, Eigen::Vector2d> getMap();
        
    
    private:
        void SmoothPath1(amp::AStar::GraphSearchResult& path, const amp::Problem2D& problem);
        int _numNodes;
        double _connectRadius;
        bool _smoothing;
        std::map<amp::Node, Eigen::Vector2d> _samples;
        amp::Graph<double> _graph;
};