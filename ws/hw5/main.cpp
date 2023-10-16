// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h" //to get the 2.b workspaces

// Include the header of the shared class
#include "Utils.h"

#include "GradientDescentBug.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    double dStar = 1.0;
    double qStar = 1.0;
    double attGain = 1.0;
    double repGain = 1.0;
    GradientDescentBug algo(dStar, qStar, attGain, repGain);

    //Problem 2a:
    /*
    Problem2D problem = HW5::getWorkspace1();
    amp::Path2D path = algo.plan(problem);
    bool success = HW5::check(path, problem, true);
    LOG("Found valid solution to Problem2a: " << (success ? "Yes!" : "No :("));
    LOG("Path length: " << path.length());
    Visualizer::makeFigure(problem, path);
    */

    //Problem 2b1:
    /*
    Problem2D problem = HW2::getWorkspace1();
    amp::Path2D path = algo.plan(problem);
    bool success = HW5::check(path, problem, true);
    LOG("Found valid solution to Problem2b1: " << (success ? "Yes!" : "No :("));
    LOG("Path length: " << path.length());
    Visualizer::makeFigure(problem, path);
    */

    //Problem 2b2
    /*
    Problem2D problem = HW2::getWorkspace2();
    amp::Path2D path = algo.plan(problem);
    bool success = HW5::check(path, problem, true);
    LOG("Found valid solution to Problem2b2: " << (success ? "Yes!" : "No :("));
    LOG("Path length: " << path.length());
    Visualizer::makeFigure(problem, path);
    */

    Visualizer::showFigures();

    HW5::grade(algo, "anpa7940@colorado.edu", argc, argv);

    return 0;
}