#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include "Utils.h"
#include "MyGridCSpace2D.h"
#include "Wavefront.h"
#include "MyLinkManipulator.h"
#include "AStar.h"

using namespace amp;


/*
class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};
*/
int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem 1
    /*
    MyPWaveFront algo;
    //Problem2D problem = HW2::getWorkspace1();
    Problem2D problem = HW2::getWorkspace2();
    //Eigen::Vector2d point(0.0, 0.0);
    //Utils::isPointInObstacles(point, problem);
    amp::Path2D path = algo.plan(problem);
    bool success = HW6::checkPointAgentPlan(path, problem, true);
    LOG("Found valid solution to Problem1: " << (success ? "Yes!" : "No :("));
    LOG("Path length: " << path.length());
    Visualizer::makeFigure(problem, path);
    */

    // Problem 2
    /*
    MyMWaveFront algo;
    Problem2D problem2a = HW6::getHW4Problem1();
    Problem2D problem2b = HW6::getHW4Problem2();
    Problem2D problem2c = HW6::getHW4Problem3();
    std::vector<double> link_lengths;
    link_lengths.push_back(1.0);
    link_lengths.push_back(1.0);
    MyLinkManipulator2D linkManip(link_lengths);

    Path2D path2a = algo.plan(linkManip,problem2a);
    Path2D path2b = algo.plan(linkManip,problem2b);
    Path2D path2c = algo.plan(linkManip,problem2c);

    bool success = HW6::checkLinkManipulatorPlan(path2a, linkManip, problem2a, true);
    LOG("Found valid solution to Problem2a: " << (success ? "Yes!" : "No :("));

    success = HW6::checkLinkManipulatorPlan(path2b, linkManip, problem2b, true);
    LOG("Found valid solution to Problem2b: " << (success ? "Yes!" : "No :("));

    success = HW6::checkLinkManipulatorPlan(path2c, linkManip, problem2c, true);
    LOG("Found valid solution to Problem2c: " << (success ? "Yes!" : "No :("));

    std::shared_ptr<MyCSpaceConstructor> ctorPtr = std::make_shared<MyCSpaceConstructor>();
    std::unique_ptr<amp::GridCSpace2D> grid_cspace = ctorPtr->construct(linkManip, problem2a);
    Visualizer::makeFigure(*grid_cspace, path2a);
    Visualizer::makeFigure(problem2a, linkManip, path2a);

    grid_cspace = ctorPtr->construct(linkManip, problem2b);
    Visualizer::makeFigure(*grid_cspace, path2b);
    Visualizer::makeFigure(problem2b, linkManip, path2b);

    grid_cspace = ctorPtr->construct(linkManip, problem2c);
    Visualizer::makeFigure(*grid_cspace, path2c);
    Visualizer::makeFigure(problem2c, linkManip, path2c);
    */

    // Problem 3
    /*
    MyAStar algo;
    amp::ShortestPathProblem problem = HW6::getEx3SPP();
    amp::LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    //DijkstraHeuristic heuristic;

    MyAStar::GraphSearchResult result = algo.search(problem, heuristic);
    
    bool success = HW6::checkGraphSearchResult(result, problem, heuristic, true);
    LOG("Found valid solution to Problem3: " << (success ? "Yes!" : "No :("));
    LOG("Path: ");
    for (amp::Node node : result.node_path) {
        LOG(node);
    }
    LOG("Total Path Cost: " << result.path_cost);
    */

    Visualizer::showFigures();
    
    MyPWaveFront algo1;
    MyMWaveFront algo2;
    MyAStar algo3;
    amp::ShortestPathProblem problem = HW6::getEx3SPP();
    MyAStar::GraphSearchResult result;
    //amp::HW6::generateAndCheck(algo3, result, problem);
    
    amp::HW6::grade(algo1, algo2, algo3, "anpa7940@colorado.edu", argc, argv);
    return 0;
}