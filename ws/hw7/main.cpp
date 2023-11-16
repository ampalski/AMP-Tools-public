#include "AMPCore.h"
#include "Utils.h"
#include "AStar.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
//#include "PRM.h"
//#include "RRT.h"

using namespace amp;
/*
void profilePRM(MyPRM algo, const amp::Problem2D& problem, std::vector<double>& success, 
  std::vector<double>& pathLength, std::vector<double>& runTime) {
    success.clear();
    pathLength.clear();
    runTime.clear();

    amp::Path2D path;
    bool pass;
    amp::Timer time(std::string(" "));
    double startTime;
    int ktr=0;

    for (int i = 0; i < 100; i++) {
        startTime = time.now();
        path = algo.plan(problem);
        pass = (path.valid && HW7::check(path, problem, false));
        if (pass) {
            success.push_back(1.0);
            pathLength.push_back(path.length());
            runTime.push_back(time.now()-startTime);
            ktr++;
        } else {
            //DEBUG("Unsuccessful Run");
            success.push_back(0.0);
        }
    }
    DEBUG("Successful runs: " << ktr);
    time.stop();
}

void profileRRT(MyRRT algo, const amp::Problem2D& problem, std::vector<double>& success, 
  std::vector<double>& pathLength, std::vector<double>& runTime) {
    success.clear();
    pathLength.clear();
    runTime.clear();

    amp::Path2D path;
    bool pass;
    amp::Timer time(std::string(" "));
    double startTime;
    int ktr=0;
    //DEBUG("Profiling");
    for (int i = 0; i < 100; i++) {
        //DEBUG("beginning of loop");
        startTime = time.now();
        path = algo.plan(problem);
        //DEBUG("Planned");
        pass = (path.valid && HW7::check(path, problem, false));
        //DEBUG("Pass: " << pass);
        if (pass) {
            success.push_back(1.0);
            pathLength.push_back(path.length());
            runTime.push_back(time.now()-startTime);
            ktr++;
        } else {
            success.push_back(0.0);
            //DEBUG("Unsuccessful Run");
        }
    }
    DEBUG("Successful runs: " << ktr);
    time.stop();
}
*/
int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());


    // Problem 1a/2a
    /*
    amp::Problem2D problem1a = HW5::getWorkspace1();
    problem1a.y_min = -3;
    problem1a.y_max = 3;
    */

    // Problem 1b/2a
    //amp::Problem2D problem1a = HW2::getWorkspace2();

    // Part i
    /*
    //MyPRM algo(200, 2.0, false);
    //MyPRM algo(200, 1.0, true);
    MyRRT algo(5000, 0.05, 0.5, 0.25);
    amp::Path2D path = algo.plan(problem1a);
    //DEBUG(path.valid);
    bool success = HW7::check(path, problem1a, true);
    //DEBUG(path.valid);
    LOG("Found valid solution to Problem: " << (success ? "Yes!" : "No :("));
    LOG("Path length: " << path.length());
    Visualizer::makeFigure(problem1a, path);
    //Visualizer::makeFigure(problem1a, algo.getGraph(), algo.getMap());
    Visualizer::makeFigure(problem1a, algo.getTree(), algo.getMap());
    */

    // Part ii, iv, 2.b
    /*
    std::list<std::vector<double>> successes;
    std::list<std::vector<double>> pathLengths;
    std::list<std::vector<double>> runTimes;
    std::vector<std::string> labels;
    std::vector<double> success;
    std::vector<double> pathLength;
    std::vector<double> runTime;
    */
    /*
    MyPRM algo1(200, 0.5, false);
    //MyPRM algo1(200, 0.5, true);
    profilePRM(algo1, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=200, r=0.5");
    DEBUG("Done 1");
    */
    /*
    //MyPRM algo2(200, 1.0, false);
    MyPRM algo2(200, 1.0, true);
    profilePRM(algo2, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=200, r=1.0");
    DEBUG("Done 2");
    */
    /*
    MyPRM algo3(200, 1.5, false);
    //MyPRM algo3(200, 1.5, true);
    profilePRM(algo3, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=200, r=1.5");
    DEBUG("Done 3");
    */
    /*
    //MyPRM algo4(200, 2.0, false);
    MyPRM algo4(200, 2.0, true);
    profilePRM(algo4, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=200, r=2.0");
    DEBUG("Done 4");
    */
    /*
    MyPRM algo5(500, 0.5, false);
    //MyPRM algo5(500, 0.5, true);
    profilePRM(algo5, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=500, r=0.5");
    DEBUG("Done 5");
    */
    /*
    //MyPRM algo6(500, 1.0, false);
    MyPRM algo6(500, 1.0, true);
    profilePRM(algo6, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=500, r=1.0");
    DEBUG("Done 6");
    */
    /*
    MyPRM algo7(500, 1.5, false);
    //MyPRM algo7(500, 1.5, true);
    profilePRM(algo7, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=500, r=1.5");
    DEBUG("Done 7");
    */
    /*
    //MyPRM algo8(500, 2.0, false);
    MyPRM algo8(500, 2.0, true);
    profilePRM(algo8, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=500, r=2.0");
    DEBUG("Done 8");
    */
    /*
    //MyPRM algo9(1000, 1.0, false);
    MyPRM algo9(1000, 1.0, true);
    profilePRM(algo9, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=1000, r=1.0");
    DEBUG("Done 9");
    */
    /*
    //MyPRM algo10(1000, 2.0, false);
    MyPRM algo10(1000, 2.0, true);
    profilePRM(algo10, problem1a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("n=1000, r=2.0");
    DEBUG("Done 10");
    */

    // Problem 2b:
    /*
    MyRRT algo(5000, 0.05, 0.5, 0.25);

    amp::Problem2D prob52a = HW5::getWorkspace1();
    profileRRT(algo, prob52a, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("HW5, Ex2a");
    DEBUG("Done 1");
    
    amp::Problem2D prob221 = HW2::getWorkspace1();
    profileRRT(algo, prob221, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("HW2, Ex2-1");
    DEBUG("Done 2");

    amp::Problem2D prob222 = HW2::getWorkspace2();
    profileRRT(algo, prob222, success, pathLength, runTime);
    successes.push_back(success);
    pathLengths.push_back(pathLength);
    runTimes.push_back(runTime);
    labels.push_back("HW2, Ex2-2");
    DEBUG("Done 3");
    */

    //Visualizer::makeBoxPlot(successes, labels, "Success Rate WS2, With Smoothing");
    //Visualizer::makeBoxPlot(pathLengths, labels, "Successful WS2 Path Lengths, With Smoothing");
    //Visualizer::makeBoxPlot(runTimes, labels, "Successful WS2 Run Times, With Smoothing", "Run Config", "Time (ms)");
    //Visualizer::makeBoxPlot(successes, labels, "Success Rate");
    //Visualizer::makeBoxPlot(pathLengths, labels, "Successful Path Lengths");
    //Visualizer::makeBoxPlot(runTimes, labels, "Successful Run Times", "Workspace", "Time (ms)");
    
    //Visualizer::showFigures();

    // Grade
    //MyPRM algoPRM(500, 2.0, true);
    //MyRRT algoRRT(5000, 0.05, 0.5, 0.25);

    //amp::HW7::grade(algoPRM, algoRRT, "anpa7940@colorado.edu", argc, argv);

    return 0;

}