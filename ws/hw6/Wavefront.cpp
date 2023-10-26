#include "Wavefront.h"
#include <queue>

std::unique_ptr<amp::GridCSpace2D> MyPWaveFront::constructDiscretizedWorkspace(const amp::Environment2D& environment) {
    // Build grid, initialize points in obstacle to be =1
    //DEBUG("Constructing Workspace");
    bool expandObstacles = true;

    double x0GridSize = 0.25;
    double x1GridSize = 0.25;
    int x0_cells = std::ceil((environment.x_max - environment.x_min)/x0GridSize)+1;
    int x1_cells = std::ceil((environment.y_max - environment.y_min)/x1GridSize)+1;

    double x0_max = environment.x_min + x0GridSize * (x0_cells -1);
    double x1_max = environment.y_min + x1GridSize * (x1_cells -1);

    std::unique_ptr<MyGridCSpace2D> cspacePtr = std::make_unique<MyGridCSpace2D>(x0_cells, x1_cells, environment.x_min, x0_max, environment.y_min, x1_max);

    // Step through each point, check if inside an obstacle or not

    double x0, x1;

    for (int i = 0; i < x0_cells; i++) {
        for (int j = 0; j < x1_cells; j++) {
            if (expandObstacles && cspacePtr->operator()(i,j)) {
                //Already checked
                continue;
            }
            x0 = environment.x_min + i * x0GridSize;
            x1 = environment.y_min + j * x1GridSize;
            bool isObst = Utils::isPointInObstacles(Eigen::Vector2d(x0, x1), environment);
            cspacePtr->operator()(i,j) = isObst;

            if (expandObstacles && isObst) {
                // Put a buffer around the obstacle to prevent close calls
                cspacePtr->operator()(std::clamp(i-1, 0, x0_cells-1), j) = true;
                cspacePtr->operator()(std::clamp(i+1, 0, x0_cells-1), j) = true;
                cspacePtr->operator()(i, std::clamp(j-1, 0, x1_cells-1)) = true;
                cspacePtr->operator()(i, std::clamp(j+1, 0, x1_cells-1)) = true;
            }
        }
    }
    return cspacePtr;
}

amp::Path2D MyPWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    //DEBUG("Entering planner");
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    int ktr = 0;

    auto[xCells, yCells] = grid_cspace.size();
    double xMin = grid_cspace.x0Bounds().first;
    double yMin = grid_cspace.x1Bounds().first;
    double xRange = grid_cspace.x0Bounds().second - xMin;
    double yRange = grid_cspace.x1Bounds().second - yMin;
    double xStep = xRange / (xCells - 1);
    double yStep = yRange / (yCells - 1);
    
    // Get the wavefront grid
    Eigen::MatrixXi wavefront = Utils::buildWavefront(q_init, q_goal, grid_cspace, false);

    // Find grid point of q_init
    auto[xind,yind] = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    int curValue, right, left, up, down, lowVal, newX, newY;

    // Continue finding the path to lower values until it reaches a wavefront value of 2
    while (wavefront(xind, yind) != 2) {
        ktr++;
        
        right = wavefront(std::clamp((int)xind + 1, 0, (int)xCells-1), yind);
        left = wavefront(std::clamp((int)xind - 1, 0, (int)xCells-1), yind);
        up = wavefront(xind, std::clamp((int)yind + 1, 0, (int)yCells-1));
        down = wavefront(xind, std::clamp((int)yind - 1, 0, (int)yCells-1));

        lowVal = INT_MAX;
        if (right > 1 && right < lowVal) {
            lowVal = right;
            newX = std::clamp((int)xind + 1, 0, (int)xCells-1);
            newY = yind;
        }
        if (left > 1 && left < lowVal) {
            lowVal = left;
            newX = std::clamp((int)xind - 1, 0, (int)xCells-1);
            newY = yind;
        }
        if (up > 1 && up < lowVal) {
            lowVal = up;
            newY = std::clamp((int)yind + 1, 0, (int)yCells-1);
            newX = xind;
        }
        if (down > 1 && down < lowVal) {
            newY = std::clamp((int)yind - 1, 0, (int)yCells-1);
            newX = xind;
        }

        xind = newX;
        yind = newY;

        path.waypoints.emplace_back(xMin + xind * xStep, yMin + yind * yStep);
        //DEBUG("New Position is " << xMin + xind * xStep <<" "<< yMin + yind * yStep);
        //PRINT_VEC2("Placed",path.waypoints.back());
        //DEBUG(ktr << ": " << wavefront(xind, yind) << ", " << path.waypoints.size());
        if (ktr > 3000) {
            break;
        }
    }
    path.waypoints.push_back(q_goal);
    return path;
}

amp::Path2D MyMWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    //DEBUG("Entering Manipulator wavefront planner");
    //PRINT_VEC2("Starting at ", q_init);
    //PRINT_VEC2("Going to ", q_goal);
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    int ktr = 0;

    auto[xCells, yCells] = grid_cspace.size();
    double xMin = grid_cspace.x0Bounds().first;
    double yMin = grid_cspace.x1Bounds().first;
    double xRange = grid_cspace.x0Bounds().second - xMin;
    double yRange = grid_cspace.x1Bounds().second - yMin;
    double xStep = xRange / (xCells - 1);
    double yStep = yRange / (yCells - 1);
    
    // Get the wavefront grid
    Eigen::MatrixXi wavefront = Utils::buildWavefront(q_init, q_goal, grid_cspace, true);
    //DEBUG("Wavefront complete");
    // Find grid point of q_init
    auto[xind,yind] = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    int curValue, right, left, up, down, lowVal, newX, newY;

    // Continue finding the path to lower values until it reaches a wavefront value of 2
    while (wavefront(xind, yind) != 2) {
        ktr++;
        if (wavefront(xind,yind) == 1) {
            //DEBUG("In obstacle at ktr="<<ktr);
            break;
        }
        if (wavefront(xind,yind) == 0) {
            //DEBUG("wavefront didn't reach init");
            break;
        }
        right = wavefront(Utils::clampWrap((int)xind + 1, 0, (int)xCells-1), yind);
        //DEBUG("Right is "<<right);
        left = wavefront(Utils::clampWrap((int)xind - 1, 0, (int)xCells-1), yind);
        //DEBUG("left is " << left);
        up = wavefront(xind, Utils::clampWrap((int)yind + 1, 0, (int)yCells-1));
        //DEBUG("up is "<<up);
        down = wavefront(xind, Utils::clampWrap((int)yind - 1, 0, (int)yCells-1));
        //DEBUG("down is "<<down);

        lowVal = INT_MAX;
        if (right > 1 && right < lowVal) {
            lowVal = right;
            newX = Utils::clampWrap((int)xind + 1, 0, (int)xCells-1);
            newY = yind;
        }
        if (left > 1 && left < lowVal) {
            lowVal = left;
            newX = Utils::clampWrap((int)xind - 1, 0, (int)xCells-1);
            newY = yind;
        }
        if (up > 1 && up < lowVal) {
            lowVal = up;
            newY = Utils::clampWrap((int)yind + 1, 0, (int)yCells-1);
            newX = xind;
        }
        if (down > 1 && down < lowVal) {
            newY = Utils::clampWrap((int)yind - 1, 0, (int)yCells-1);
            newX = xind;
        }
        if (lowVal == INT_MAX);
        xind = newX;
        yind = newY;

        path.waypoints.emplace_back(xMin + xind * xStep, yMin + yind * yStep);
        //DEBUG("New Position is " << xMin + xind * xStep <<" "<< yMin + yind * yStep);
        //PRINT_VEC2("Placed",path.waypoints.back());
        //DEBUG(ktr << ": " << wavefront(xind, yind) << ", " << path.waypoints.size());
        if (ktr > 1000) {
            break;
        }
    }
    path.waypoints.push_back(q_goal);
    //amp::unwrapPath(path, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2*M_PI,2*M_PI));
    amp::unwrapPath(path, Eigen::Vector2d(-M_PI, -M_PI), Eigen::Vector2d(M_PI,M_PI));
    return path;
}