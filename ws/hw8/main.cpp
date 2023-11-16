#include "AMPCore.h"
#include "Utils.h"
#include "AStar.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include "RRT.h"
#include "MultiRRT.h"
#include <numeric>

using namespace amp;

void profile(MyCentralMultiRRT algo, const amp::MultiAgentProblem2D& problem, 
  std::vector<double>& runTime, std::vector<double>& treeSize) {
    runTime.clear();
    treeSize.clear();

    amp::MultiAgentPath2D path;
    bool pass;
    amp::Timer time(std::string(" "));
    double startTime;
    int ktr = 0;

    for (int i = 0; i < 100; i++) {
        startTime = time.now();
        path = algo.plan(problem);
        pass = (path.valid && HW8::check(path, problem, false));
        if (pass) {
            treeSize.push_back(algo.getTree().nodes().size());
            runTime.push_back(time.now()-startTime);
            ktr++;
            //DEBUG(ktr);
        }
    }
    DEBUG("Successful runs: " << ktr);
    time.stop();
}

void profile(MyDecentralMultiRRT algo, const amp::MultiAgentProblem2D& problem, 
  std::vector<double>& runTime) {
    runTime.clear();

    amp::MultiAgentPath2D path;
    bool pass;
    amp::Timer time(std::string(" "));
    double startTime;
    int ktr = 0;

    for (int i = 0; i < 100; i++) {
        startTime = time.now();
        path = algo.plan(problem);
        pass = (path.valid && HW8::check(path, problem, false));
        if (pass) {
            runTime.push_back(time.now()-startTime);
            ktr++;
            //DEBUG(ktr);
        }
    }
    DEBUG("Successful runs: " << ktr);
    time.stop();
}

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem 1b/2b: 
    /*
    int m = 6;
    amp::MultiAgentProblem2D problem = HW8::getWorkspace1(m);
    MyCentralMultiRRT algo(7500, 0.05, 0.5, 0.25);
    //MyDecentralMultiRRT algo(7500, 0.05, 0.5, 0.25);
    amp::MultiAgentPath2D path = algo.plan(problem);
    DEBUG(path.valid);
    DEBUG(path.numAgents());
    //std::vector<std::vector<Eigen::Vector2d>> collision_states;
    bool success = HW8::check(path, problem, true);
    LOG("Found valid solution to Problem: " << (success ? "Yes!" : "No :("));
    Visualizer::makeFigure(problem, path);
    */
    
    // Checking random cases for problems
    /*
    amp::MultiAgentProblem2D problem;
    //MyCentralMultiRRT algo(7500, 0.05, 0.5, 0.25);
    MyDecentralMultiRRT algo(7500, 0.05, 0.5, 0.25);
    amp::Timer time(std::string(" "));
    double startTime;
    amp::MultiAgentPath2D path;
    for (int i = 0; i < 100; i++) {
        startTime = time.now();
        bool success = HW8::generateAndCheck(algo, path, problem, true);
        if (time.now() - startTime > 10000) {
            Visualizer::makeFigure(problem, path);
        }
    }
    */
    
    //LOG("Found valid solution to Problem: " << (success ? "Yes!" : "No :("));
    //Visualizer::makeFigure(problem, path);

    // Problem 1c-e/2c-e:
    /*
    int m = 2;
    std::list<std::vector<double>> treeSizes;
    std::list<std::vector<double>> runTimes;
    std::vector<std::string> labels;
    std::vector<double> treeSize;
    std::vector<double> runTime;
    std::vector<double> treeSizeMeans;
    std::vector<double> runTimeMeans;

    amp::MultiAgentProblem2D problem1 = HW8::getWorkspace1(m);
    //MyCentralMultiRRT algo1(7500, 0.05, 0.5, 0.25);
    MyDecentralMultiRRT algo(7500, 0.05, 0.5, 0.25);
    //profile(algo1, problem1, runTime, treeSize);
    profile(algo, problem1, runTime);
    //treeSizes.push_back(treeSize);
    //treeSizeMeans.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
    runTimes.push_back(runTime);
    runTimeMeans.push_back(std::accumulate(runTime.begin(), runTime.end(), 0.0) / runTime.size());
    labels.push_back("m = 2");
    DEBUG("Done 2");
    
    m = 3;
    amp::MultiAgentProblem2D problem2 = HW8::getWorkspace1(m);
    //MyCentralMultiRRT algo2(7500, 0.05, 0.5, 0.25);
    //profile(algo2, problem2, runTime, treeSize);
    profile(algo, problem2, runTime);
    //treeSizes.push_back(treeSize);
    //treeSizeMeans.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
    runTimes.push_back(runTime);
    runTimeMeans.push_back(std::accumulate(runTime.begin(), runTime.end(), 0.0) / runTime.size());
    labels.push_back("m = 3");
    DEBUG("Done 3");

    m = 4;
    amp::MultiAgentProblem2D problem3 = HW8::getWorkspace1(m);
    //MyCentralMultiRRT algo3(7500, 0.05, 0.5, 0.25);
    //profile(algo3, problem3, runTime, treeSize);
    profile(algo, problem3, runTime);
    //treeSizes.push_back(treeSize);
    //treeSizeMeans.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
    runTimes.push_back(runTime);
    runTimeMeans.push_back(std::accumulate(runTime.begin(), runTime.end(), 0.0) / runTime.size());
    labels.push_back("m = 4");
    DEBUG("Done 4");

    m = 5;
    amp::MultiAgentProblem2D problem4 = HW8::getWorkspace1(m);
    //MyCentralMultiRRT algo4(7500, 0.05, 0.5, 0.25);
    //profile(algo4, problem4, runTime, treeSize);
    profile(algo, problem4, runTime);
    //treeSizes.push_back(treeSize);
    //treeSizeMeans.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
    runTimes.push_back(runTime);
    runTimeMeans.push_back(std::accumulate(runTime.begin(), runTime.end(), 0.0) / runTime.size());
    labels.push_back("m = 5");
    DEBUG("Done 5");

    m = 6;
    amp::MultiAgentProblem2D problem5 = HW8::getWorkspace1(m);
    //MyCentralMultiRRT algo5(7500, 0.05, 0.5, 0.25);
    //profile(algo5, problem5, runTime, treeSize);
    profile(algo, problem5, runTime);
    //treeSizes.push_back(treeSize);
    //treeSizeMeans.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
    runTimes.push_back(runTime);
    runTimeMeans.push_back(std::accumulate(runTime.begin(), runTime.end(), 0.0) / runTime.size());
    labels.push_back("m = 6");
    DEBUG("Done 6");
    
    //Visualizer::makeBoxPlot(treeSizes, labels, "Successful Tree Sizes", "# of Agents", "# of Total Nodes");
    Visualizer::makeBoxPlot(runTimes, labels, "Successful Run Times", "# of Agents", "Time (ms)");

    //Visualizer::makeBarGraph(treeSizeMeans, labels, "Mean Tree Size vs # of Agents", "# of Agents", "# of Total Nodes");
    Visualizer::makeBarGraph(runTimeMeans, labels, "Mean Run Times vs # of Agents", "# of Agents", "Time (ms)");
    */

    Visualizer::showFigures();
    // Grade
    MyCentralMultiRRT algo1(7500, .05, 0.5, 0.25);
    MyDecentralMultiRRT algo2(7500, 0.05, 0.5, 0.25);
    amp::HW8::grade(algo1, algo2, "anpa7940@colorado.edu", argc, argv);
    //bool valid = HW8::generateAndCheck(algo1);

    return 0;

}