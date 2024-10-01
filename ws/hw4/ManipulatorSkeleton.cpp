#include "ManipulatorSkeleton.h"

void printJointPosition(std::vector<Eigen::Vector2d> positions)
{
    for (int i = 0; i < positions.size(); i++)
    {
        std::cout << "{" << positions[i][0] << ", " << positions[i][1] << "} ";
    }
    std::cout << std::endl;
}

// Function to compute FABRIK (Forward And Backward Reaching Inverse Kinematics)
auto fabrik(const std::vector<double>& a, const Eigen::Vector2d& end_location, double tol = 1e-4, int max_iters = 1000) {
    int n = a.size();
    double total_length = 0.0;
    for (double len : a) total_length += len;

    if (end_location.norm() > total_length) {
        throw std::invalid_argument("The links are not long enough to reach the desired point!");
    }

    // Zero out initial position (start at a flat chain along x-axis)
    std::vector<Eigen::Vector2d> link_ends(n + 1, Eigen::Vector2d::Zero());
    for (int i = 1; i <= n; ++i) {
        link_ends[i] = link_ends[i - 1] + Eigen::Vector2d(a[i - 1], 0.0);
    }

    // std::cout << "zero " << " ";
    // printJointPosition(link_ends);

    for (int iter = 1; iter < max_iters; ++iter) {
        // "Forward" step: set the end-effector to the desired location
        link_ends[n] = end_location;

        for (int i = n - 1; i >= 0; --i) {
            double r = (link_ends[i] - link_ends[i + 1]).norm();
            if (r < 1e-6) {
                link_ends[i] += Eigen::Vector2d(0.01, 0.01);
                r = (link_ends[i] - link_ends[i + 1]).norm();
            }
            double lamb = a[i] / r;
            // std::cout << "f inner " << i << " " << r << std::endl;
            link_ends[i] = (1 - lamb) * link_ends[i + 1] + lamb * link_ends[i];
        }
        // std::cout << "f" << iter << " ";
        // printJointPosition(link_ends);

        // "Backward" step: re-fix the base at (0,0)
        link_ends[0] = Eigen::Vector2d::Zero();

        for (int i = 0; i < n; ++i) {
            double r = (link_ends[i] - link_ends[i + 1]).norm();
            if (r < 1e-6) {
                link_ends[i + 1] += Eigen::Vector2d(0.01, 0.01);
                r = (link_ends[i] - link_ends[i + 1]).norm();
            }
            double lamb = a[i] / r;
            link_ends[i + 1] = (1 - lamb) * link_ends[i] + lamb * link_ends[i + 1];
        }

        // std::cout << "b" << iter << " ";
        // printJointPosition(link_ends);

        // If the end-effector is close enough to the target, break
        if ((link_ends[n] - end_location).norm() < tol) {
            break;
        }
    }

    return link_ends;
}

// Function to calculate angles between the links given the link positions
amp::ManipulatorState get_angles_from_link_ends(const std::vector<Eigen::Vector2d>& link_ends) {
    int n = link_ends.size() - 1;
    amp::ManipulatorState angles(n);

    std::vector<Eigen::Vector2d> padded_link_ends(n + 2, Eigen::Vector2d(-1, 0));
    for (int i = 0; i < link_ends.size(); ++i) {
        padded_link_ends[i + 1] = link_ends[i];
    }

    for (int i = 0; i < n; ++i) {
        Eigen::Vector2d prev_link = padded_link_ends[i + 1] - padded_link_ends[i];
        Eigen::Vector2d next_link = padded_link_ends[i + 2] - padded_link_ends[i + 1];

        double dot = prev_link.dot(next_link);
        double cross = prev_link.x() * next_link.y() - prev_link.y() * next_link.x();
        angles[i] = std::atan2(cross, dot);

        // Ensure the angle is positive
        if (angles[i] < 0) {
            angles[i] += 2 * M_PI;
        }
    }

    return angles;
}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0)};
    
    auto link_lengths = this->getLinkLengths();

    int N = state.size();  // Number of links

    // Validate the input lengths
    if (link_lengths.size() != N) {
        throw std::invalid_argument("The size of link lengths must match the number of angles.");
    }

    // Insert a zero at the beginning
    link_lengths.insert(link_lengths.begin(), 0);

    // Append zero angle at the end to represent the end-effector
    Eigen::VectorXd all_angles(N + 1);
    all_angles.head(N) = state;
    all_angles[N] = 0.0;

    // Calculate transformation matrices for each link
    std::vector<Eigen::Matrix3d> transforms(N + 1);
    for (int i = 0; i < N+1; ++i) {
        transforms[i] << std::cos(all_angles[i]), -std::sin(all_angles[i]), link_lengths[i],
                            std::sin(all_angles[i]),  std::cos(all_angles[i]), 0,
                            0, 0, 1;
    }

    // Combine the transformations step by step
    Eigen::Matrix3d comb = Eigen::Matrix3d::Identity();
    std::vector<Eigen::Matrix3d> combined_transform(N + 1);
    for (int i = 0; i < N+1; ++i) {
        comb = comb * transforms[i];
        combined_transform[i] = comb;
    }

    // Calculate the positions of the link ends
    std::vector<Eigen::Vector2d> link_ends(N + 1);
    for (int i = 0; i < N+1; ++i) {
        Eigen::Vector3d link_end = combined_transform[i] * Eigen::Vector3d(0, 0, 1);
        link_ends[i] = link_end.head<2>();  // Extract x, y coordinates
        // std::cout << link_ends[i] << std::endl;
    }
    
    return link_ends[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {

    // Special case with analytical solution
    if (this->getLinkLengths().size() == 2)
    {
        auto& a = getLinkLengths();

        // Calculate the second angle theta2 using the law of cosines
        double cos_theta2 = (end_effector_location.squaredNorm() - a[0] * a[0] - a[1] * a[1]) / (2 * a[0] * a[1]);
        double theta2 = std::acos(cos_theta2);
        double sin_theta2 = std::sin(theta2);

        // Calculate the first angle theta1
        double cos_theta1 = (end_effector_location[0]*(a[0]+a[1]*cos_theta2) + end_effector_location[1]*a[1]*sin_theta2)/end_effector_location.squaredNorm();
        double sin_theta1 = (end_effector_location[1]*(a[0]+a[1]*cos_theta2) - end_effector_location[0]*a[1]*sin_theta2)/end_effector_location.squaredNorm();

        double theta1 = std::atan2(sin_theta1, cos_theta1);

        // Return the joint angles
        return Eigen::Vector2d{theta1, theta2};
    }

    auto joint_locations = fabrik(this->getLinkLengths(), end_effector_location, 1e-10, 1000);
    return get_angles_from_link_ends(joint_locations);
}