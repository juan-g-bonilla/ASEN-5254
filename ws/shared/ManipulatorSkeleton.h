#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
// Derive the amp::LinkManipulator2D class
class MyManipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        using LinkManipulator2D::LinkManipulator2D;

        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        amp::ManipulatorState getConfigurationFromIK2Link(const Eigen::Vector2d& end_effector_location, bool first = true) const;

};