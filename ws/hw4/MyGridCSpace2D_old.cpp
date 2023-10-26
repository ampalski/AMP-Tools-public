/*#include "MyGridCSpace2D.h"
#include "Utils.h"

bool MyGridCSpace2D::inCollision(double x0, double x1) const {
    //grab the DenseArray2D and find x0 and x1 within it, linear interp
    //get x and y min-max values, and how many discrete points are between them, which gives the x-y points for the grid
    //then find grid points that are closest to x and y =x1, return that point
    //DEBUG("Need to find (" <<x0<<", "<<x1<<")");
    std::pair numCells = MyGridCSpace2D::size();
    double x0low = MyGridCSpace2D::m_x0_bounds.first;
    double x0max = MyGridCSpace2D::m_x0_bounds.second;
    double x0Step = (x0max - x0low) / (numCells.first-1);

    double x1low = MyGridCSpace2D::m_x1_bounds.first;
    double x1max = MyGridCSpace2D::m_x1_bounds.second;
    double x1Step = (x1max - x1low) / (numCells.second-1);

    // double check that x0 and x1 are within the CSpace
    if (x0 < x0low || x0 > MyGridCSpace2D::m_x0_bounds.second || 
            x1 < x1low || x1 > MyGridCSpace2D::m_x1_bounds.second) {
        return false;
    }

    // find the nearest discrete point
    int x0ind = std::round((x0 - x0low) / x0Step);
    x0ind = x0ind > numCells.first ? numCells.first : x0ind;
    int x1ind = std::round((x0 - x0low) / x0Step);
    x1ind = x1ind > numCells.second ? numCells.second : x1ind;
    //DEBUG("found inds (" <<x0ind<<", "<<x1ind<<")");
    //DEBUG("point is (" <<(x0low+x0ind*x0Step)<<", "<<(x0low+x0ind*x0Step)<<")");

    // return the data at that index
    return MyGridCSpace2D::operator()(x0ind, x1ind);
}

std::unique_ptr<amp::GridCSpace2D> MyCSpaceConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    //Build grid, step through grid point by point
    //test forward kinematics on each point against known obstacles
    //If collision free, that point is false, else true

    // Two revolute joints with range [0:2pi]
    int x0_cells = 1000;
    int x1_cells = 1000;
    double x0_min = 0.0;
    double x1_min = 0.0;
    double x0_step = 2*M_PI / (x0_cells-1);
    double x1_step = 2*M_PI / (x1_cells-1);
    double x0_max = 2*M_PI;
    double x1_max = 2*M_PI;

    std::unique_ptr<MyGridCSpace2D> cspacePtr = std::make_unique<MyGridCSpace2D>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

    // Step through the grid, finding the c-space state at each point
    // Get the joint locations for the manipulator
    // Do collision checking of each link against the environment
    double x0, x1;
    amp::ManipulatorState state;
    Eigen::Vector2d joint0 = manipulator.getJointLocation(state, 0);
    Eigen::Vector2d joint1, joint2;
    bool link1, link2;
    for (int i = 0; i < x0_cells; i++) {
        state.clear();
        x0 = x0_min + i * x0_step;
        state.push_back(x0);
        state.push_back(0.0);
        for (int j = 0; j < x1_cells; j++) {
            x1 = x1_min + j * x0_step;
            state[1] = x1;
            joint1 = manipulator.getJointLocation(state,1);
            joint2 = manipulator.getJointLocation(state,2);

            link1 = Utils::checkStep(joint0, joint1, env);
            if (link1) {
                cspacePtr->operator()(i, j) = true;
                continue;
            } 
            link2 = Utils::checkStep(joint1, joint2, env);

            cspacePtr->operator()(i, j) = (link1 || link2);
        }
    }

    return cspacePtr;
}
*/