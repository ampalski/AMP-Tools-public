// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "Utils.h"
#include "MyLinkManipulator.h"
#include "MyGridCSpace2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem 1
    /*
    std::vector<Eigen::Vector2d> vertices;
    vertices.push_back(Eigen::Vector2d(0.0, 0.0));
    vertices.push_back(Eigen::Vector2d(1.0, 2.0));
    vertices.push_back(Eigen::Vector2d(0.0, 2.0));
    amp::Polygon obstacle(vertices);
    amp::Polygon robot(vertices);
    */

    //Problem 1a
    /*
    amp::Polygon csObstacle = Utils::CSObstConvPolySingle(obstacle, robot, 0.0);
    for (Eigen::Vector2d vertex : csObstacle.verticesCCW()) {
        PRINT_VEC2("Vertex ", vertex);
    }
    */

    //Problem 1b
    /*
    std::vector<double> rotations;
    double current = 0.0;
    while (current <= 2 * M_PI) {
        rotations.push_back(current);
        current += M_PI / 6;
    }
    std::vector<amp::Polygon> polygons = Utils::CSObstConvPolyRotate(obstacle, robot, rotations);
    Visualizer::makeFigure(polygons, rotations);
    */

    //Problem 2a
    /*
    std::vector<double> link_lengths;
    link_lengths.push_back(0.5);
    link_lengths.push_back(1.0);
    link_lengths.push_back(0.5);
    //DEBUG("link_lengths is " << link_lengths.size());
    MyLinkManipulator2D linkManip(link_lengths);
    amp::ManipulatorState state;
    state.push_back(M_PI/6);
    state.push_back(M_PI/3);
    state.push_back(7 * M_PI / 4);
    //DEBUG("state is " << state[0] << " " << state[1] << " " << state[2]);
    //Eigen::Vector2d temp = linkManip.getJointLocation(state, 1);
    //PRINT_VEC2("Manual state is ", temp);
    Visualizer::makeFigure(linkManip, state);
    */

    //Problem 2b
    /*
    std::vector<double> link_lengths;
    link_lengths.push_back(1.0);
    link_lengths.push_back(0.5);
    link_lengths.push_back(1.0);
    MyLinkManipulator2D linkManip(link_lengths);
    const Eigen::Vector2d endloc(2.0, 0.0);
    ManipulatorState state = linkManip.getConfigurationFromIK(endloc);
    DEBUG("state is " << state[0] << " " << state[1] << " " << state[2]);
    Visualizer::makeFigure(linkManip, state);
    */

    //Problem 3
    MyCSpaceConstructor constructor;
    std::vector<double> link_lengths;
    link_lengths.push_back(1.0);
    link_lengths.push_back(1.0);
    MyLinkManipulator2D linkManip(link_lengths);
    std::vector<Eigen::Vector2d> obstacle;
    //3a
    //Define the polygons and environments
    
    obstacle.push_back(Eigen::Vector2d(0.25, 0.25));
    obstacle.push_back(Eigen::Vector2d(0.0, 0.75));
    obstacle.push_back(Eigen::Vector2d(-0.25, 0.25));
    Polygon obsA(obstacle);
    Environment2D prob3a;
    prob3a.x_max = 2.0;
    prob3a.x_min = -2.0;
    prob3a.y_min = -2.0;
    prob3a.y_max = 2.0;
    prob3a.obstacles.push_back(obsA);
    std::unique_ptr<GridCSpace2D> cspace = constructor.construct(linkManip, prob3a);
    Visualizer::makeFigure(prob3a);
    Visualizer::makeFigure(*cspace);
    double win = amp::HW4::checkCSpace(*cspace, linkManip, prob3a, 10000, true);

    //3b
    obstacle.push_back(Eigen::Vector2d(-0.25, 1.1));
    obstacle.push_back(Eigen::Vector2d(-0.25, 2.0));
    obstacle.push_back(Eigen::Vector2d(0.25, 2.0));
    obstacle.push_back(Eigen::Vector2d(0.25, 1.1));
    Polygon O1(obstacle);
    obstacle.clear();
    /*
    obstacle.push_back(Eigen::Vector2d(-2.0, -2.0));
    obstacle.push_back(Eigen::Vector2d(-2.0, -1.8));
    obstacle.push_back(Eigen::Vector2d(2.0, -1.8));
    obstacle.push_back(Eigen::Vector2d(2.0, -2.0));
    Polygon O2(obstacle);
    Environment2D prob3b;
    prob3b.x_max = 2.0;
    prob3b.x_min = -2.0;
    prob3b.y_min = -2.0;
    prob3b.y_max = 2.0;
    prob3b.obstacles.push_back(O1);
    prob3b.obstacles.push_back(O2);
    std::unique_ptr<GridCSpace2D> cspace = constructor.construct(linkManip, prob3b);
    Visualizer::makeFigure(prob3b);
    Visualizer::makeFigure(*cspace);
    */

    //3c
    /*
    obstacle.clear();
    obstacle.push_back(Eigen::Vector2d(-2.0, -0.5));
    obstacle.push_back(Eigen::Vector2d(-2.0, -0.3));
    obstacle.push_back(Eigen::Vector2d(2.0, -0.3));
    obstacle.push_back(Eigen::Vector2d(2.0, -0.5));
    Polygon O2(obstacle);
    Environment2D prob3c;
    prob3c.x_max = 2.0;
    prob3c.x_min = -2.0;
    prob3c.y_min = -2.0;
    prob3c.y_max = 2.0;
    prob3c.obstacles.push_back(O1);
    prob3c.obstacles.push_back(O2);
    std::unique_ptr<GridCSpace2D> cspace = constructor.construct(linkManip, prob3c);
    //Visualizer::makeFigure(prob3c);
    //Visualizer::makeFigure(*cspace);
    //cspace->inCollision(M_PI, M_PI);
    */
    Visualizer::showFigures();
    
    // Grade method
    //amp::HW4::grade<MyLinkManipulator2D>(constructor, "anpa7940@colorado.edu", argc, argv);
    return 0;
}