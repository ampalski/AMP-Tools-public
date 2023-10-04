// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "Utils.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem 1a
    std::vector<Eigen::Vector2d> vertices;
    vertices.push_back(Eigen::Vector2d(0.0, 0.0));
    vertices.push_back(Eigen::Vector2d(1.0, 2.0));
    vertices.push_back(Eigen::Vector2d(0.0, 2.0));
    amp::Polygon obstacle(vertices);
    amp::Polygon robot(vertices);
    amp::Polygon csObstacle = Utils::CSObstConvPolyTranslate(obstacle, robot);
    for (Eigen::Vector2d vertex : csObstacle.verticesCCW()) {
        PRINT_VEC2("Vertex ", vertex);
    }

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}