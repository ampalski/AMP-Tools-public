#pragma once

#include "hw/HW6.h"
#include "MyGridCSpace2D.h"
#include "Utils.h"
#include "tools/Algorithms.h"
#include "tools/Graph.h"

class MyAStar : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

struct DijkstraHeuristic : public amp::SearchHeuristic {
    DijkstraHeuristic() {}

    virtual double operator()(amp::Node node) const override {return 0.0;}
};