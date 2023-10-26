#pragma once

#include "hw/HW6.h"
#include "MyGridCSpace2D.h"
#include "Utils.h"

class MyPWaveFront : public amp::PointWaveFrontAlgorithm {
    public:
        /// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
        /// able to use that code here to construct a discretized C-space for a point agent.
        /// @param environment Workspace and/or C-space (point agent)
        /// @return Unique pointer to a GridCSpace2D object (see HW4)
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
        
};

class MyMWaveFront : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        //Default ctor
        MyMWaveFront() 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceConstructor>()) {}
        
        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};
/*
has protected std::shared_ptr<GridCSpace2DConstructor> m_c_space_constructor
inherits from WaveFrontAlgorithm
which gets
amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace)

also inherits LinkManipulatorMotionPlanner2D
which gets
plan (already implemented in HW6.h)
*/