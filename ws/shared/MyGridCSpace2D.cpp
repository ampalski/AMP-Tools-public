#include "MyGridCSpace2D.h"
#include "Utils.h"

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
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

    // find the nearest discrete point
    int x0ind = std::round((x0 - x0low) / x0Step);
    x0ind = std::clamp(x0ind, 0, (int)numCells.first);
    int x1ind = std::round((x1 - x1low) / x1Step);
    x1ind = std::clamp(x1ind, 0, (int)numCells.second);
    //DEBUG("found inds (" <<x0ind<<", "<<x1ind<<")");
    //DEBUG("point is (" <<(x0low+x0ind*x0Step)<<", "<<(x1low+x1ind*x1Step)<<")");

    // return the data at that index
    return std::pair<std::size_t, std::size_t>(x0ind, x1ind);
}

std::unique_ptr<amp::GridCSpace2D> MyCSpaceConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    //Build grid, step through grid point by point
    //test forward kinematics on each point against known obstacles
    //If collision free, that point is false, else true

    bool expandObstacles = true;

    // Two revolute joints with range [0:2pi]
    int x0_cells = 200;
    int x1_cells = 200;
    //double x0_min = 0.0;
    double x0_min = -M_PI;
    //double x1_min = 0.0;
    double x1_min = -M_PI;
    double x0_step = 2*M_PI / (x0_cells-1);
    double x1_step = 2*M_PI / (x1_cells-1);
    //double x0_max = 2*M_PI;
    //double x1_max = 2*M_PI;
    double x0_max = M_PI;
    double x1_max = M_PI;

    std::unique_ptr<MyGridCSpace2D> cspacePtr = std::make_unique<MyGridCSpace2D>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

    // Step through the grid, finding the c-space state at each point
    // Get the joint locations for the manipulator
    // Do collision checking of each link against the environment
    double x0, x1;
    amp::ManipulatorState state(2);
    state << 0.0, 0.0;
    Eigen::Vector2d joint0 = manipulator.getJointLocation(state, 0);
    Eigen::Vector2d joint1, joint2;
    bool link1, link2;
    for (int i = 0; i < x0_cells; i++) {
        //state.clear(); // this is a VectorXd now, not std::vector, so need to fix
        x0 = x0_min + i * x0_step;
        state(0) = x0;
        for (int j = 0; j < x1_cells; j++) {
            x1 = x1_min + j * x0_step;
            state(1) = x1;
            joint1 = manipulator.getJointLocation(state,1);
            joint2 = manipulator.getJointLocation(state,2);

            link1 = Utils::checkStep(joint0, joint1, env);
            if (link1) {
                cspacePtr->operator()(i, j) = true;

                if (expandObstacles) {
                    cspacePtr->operator()(Utils::clampWrap(i+1,0, x0_cells-1),j) = true;
                    cspacePtr->operator()(Utils::clampWrap(i-1,0, x0_cells-1),j) = true;
                    cspacePtr->operator()(i, Utils::clampWrap(j+1,0, x1_cells-1)) = true;
                    cspacePtr->operator()(i, Utils::clampWrap(j-1,0, x1_cells-1)) = true;
                }

                continue;
            } 
            link2 = Utils::checkStep(joint1, joint2, env);

            cspacePtr->operator()(i, j) = (link1 || link2);
            if (expandObstacles && (link1 || link2)) {
                cspacePtr->operator()(Utils::clampWrap(i+1,0, x0_cells-1),j) = true;
                cspacePtr->operator()(Utils::clampWrap(i-1,0, x0_cells-1),j) = true;
                cspacePtr->operator()(i, Utils::clampWrap(j+1,0, x1_cells-1)) = true;
                cspacePtr->operator()(i, Utils::clampWrap(j-1,0, x1_cells-1)) = true;
            }
        }
    }

    return cspacePtr;
}
