#pragma once

#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <cmath>
#include <stdexcept>
#include "tools/LinkManipulator.h"

#include "tools/Serializer.h"


class MyLinkManipulator2D : public amp::LinkManipulator2D {
    public:
        /// @brief Construct 2-link manipulator. The base location is set to (0.0, 0.0), each link_length is 1.0
        MyLinkManipulator2D();

        /// @brief Construct from an array of link lengths. The base location is set to (0.0, 0.0)
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        MyLinkManipulator2D(const std::vector<double>& link_lengths);

        /// @brief Construct from an array of link lengths.
        /// @param base_location Custom base location of the manipulator
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        MyLinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        /******* User Implemented Methods ********/

        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        /*****************************************/
        /// @brief Virtual dtor
        ~MyLinkManipulator2D() {}
};
