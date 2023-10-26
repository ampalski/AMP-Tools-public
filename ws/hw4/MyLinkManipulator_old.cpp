/*#include "MyLinkManipulator.h"
#include "Utils.h"

MyLinkManipulator2D::MyLinkManipulator2D() {
    m_base_location = Eigen::Vector2d(0.0, 0.0);
    m_link_lengths.push_back(1.0);
    m_link_lengths.push_back(1.0);
}

MyLinkManipulator2D::MyLinkManipulator2D(const std::vector<double>& link_lengths) {
    m_base_location = Eigen::Vector2d(0.0, 0.0);

    m_link_lengths.clear();

    for (double length : link_lengths) {
        m_link_lengths.push_back(length);
    }
}

MyLinkManipulator2D::MyLinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths) {
    m_base_location = Eigen::Vector2d(base_location(0), base_location(1));
    for (double length : link_lengths) {
        m_link_lengths.push_back(length);
    }
}


Eigen::Vector2d MyLinkManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    //Use ManipulatorState[joint_index-1] for the angles
    //Use the summation form given the patterns of the transformation matrix
    //DEBUG("Joint index " << joint_index);
    Eigen::Vector2d jointLocation(0.0, 0.0);
    double anglei;
    for (int i = 0; i < joint_index; i++) {
        anglei = 0.0;
        for (int j = 0; j <= i; j++) {
            //DEBUG("j is " << j << " state[j] is " << state[j]);
            anglei += state[j];
        }
        //DEBUG("Step "<< i << " has angle " << anglei);
        jointLocation(0) = jointLocation(0) + m_link_lengths[i] * cos(anglei);
        jointLocation(1) = jointLocation(1) + m_link_lengths[i] * sin(anglei);
        //PRINT_VEC2("Intermediate joint loc ", jointLocation);
    }
    //PRINT_VEC2("Link at ", (jointLocation + m_base_location));
    return (jointLocation + m_base_location);
}

amp::ManipulatorState MyLinkManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    //Currently only works for 3-link robots
    amp::ManipulatorState config;
    // Assume first link is in direction of goal

    double theta1 = atan2(end_effector_location(1), end_effector_location(0));

    // Check the 2-3 links can reach goal

    Eigen::Vector2d link1Loc(m_link_lengths[0]*cos(theta1), m_link_lengths[0]*sin(theta1));
    double d = (end_effector_location - link1Loc).norm();

    if (d > (m_link_lengths[1] + m_link_lengths[2])) {
        //PRINT_VEC2("Goal too far away for ", end_effector_location);
        config.push_back(0.0);
        config.push_back(0.0);
        config.push_back(0.0);
        return config;
        throw std::logic_error("Not possible");
    }
    int ktr = 0;
    while (d < abs(m_link_lengths[1] - m_link_lengths[2])) {
        ktr++;
        theta1 += (M_PI/250);
        link1Loc = {m_link_lengths[0]*cos(theta1), m_link_lengths[0]*sin(theta1)};
        d = (end_effector_location - link1Loc).norm();

        if (ktr > 550) {
            //PRINT_VEC2("No valid config for ", end_effector_location);
            config.push_back(0.0);
            config.push_back(0.0);
            config.push_back(0.0);
            return config;
            throw std::logic_error("Not possible, no valid configuation");
        }
    }

    // reconfigure to new end-effector-location, make first joint the new base
    Eigen::Vector2d newGoal = Utils::rotateVec(end_effector_location - link1Loc, -theta1 * 180 / M_PI);
    //PRINT_VEC2("New Goal is ", newGoal);
    // run basic 2-link IK
    double c2, s2, c3, s3;
    c3 = ((pow(newGoal(0),2) + pow(newGoal(1),2)) - (pow(m_link_lengths[1],2) + pow(m_link_lengths[2],2)));
    //DEBUG("c3 is " << c3);
    c3 /= (2 * m_link_lengths[1] * m_link_lengths[2]);

    s3 = sqrt(1 - pow(c3, 2));

    c2 = newGoal(0) * (m_link_lengths[1] + m_link_lengths[2] * c3) + newGoal(1) * m_link_lengths[2] * s3;
    c2 /= (pow(newGoal(0),2) + pow(newGoal(1),2));

    s2 = newGoal(1) * (m_link_lengths[1] + m_link_lengths[2] * c3) - newGoal(0) * m_link_lengths[2] * s3;
    s2 /= (pow(newGoal(0),2) + pow(newGoal(1),2));

    double theta2 = atan2(s2, c2);
    double theta3 = atan2(s3, c3);

    config.push_back(theta1);
    config.push_back(theta2);
    config.push_back(theta3);

    return config;
}*/