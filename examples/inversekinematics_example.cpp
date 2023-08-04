#include <chrono>
#include <iostream>

#include "../include/inversekinematics.hpp"
#include "../include/kinematics.hpp"
#include "../include/model.hpp"
#include "../include/parser.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Load model
    const int n_joints = 5;
    auto link_5        = import_urdf<double, n_joints>("../data/urdfs/5_link.urdf");
    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = link_5.random_configuration();
    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "end_effector";
    std::string source_link_name = "ground";
    Hst_desired                  = forward_kinematics(link_5, q_random, target_link_name, source_link_name);
    // Compute the inverse kinematics for the random desired transform
    auto q0 = link_5.home_configuration();
    InverseKinematicsOptions<double, n_joints> options;
    options.max_iterations = 2000;
    options.tolerance      = 1e-7;
    options.method         = InverseKinematicsMethod::LEVENBERG_MARQUARDT;
    auto start             = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, n_joints, 1> q_solution =
        inverse_kinematics<double, n_joints>(link_5, target_link_name, source_link_name, Hst_desired, q0, options);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = forward_kinematics(link_5, q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    std::cout << "IK error: " << homogeneous_error(Hst_solution, Hst_desired).norm() << std::endl;
    std::cout << "Time taken by inverse kinematics: " << duration.count() << " microseconds" << std::endl;
}
