#include <chrono>
#include <iostream>
#include <map>

#include "../include/inversekinematics.hpp"
#include "../include/kinematics.hpp"
#include "../include/model.hpp"
#include "../include/parser.hpp"

using namespace tinyrobotics;

std::string method_to_string(InverseKinematicsMethod method) {
    static std::map<InverseKinematicsMethod, std::string> method_names = {
        {InverseKinematicsMethod::JACOBIAN, "JACOBIAN"},
        {InverseKinematicsMethod::NLOPT, "NLOPT"},
        {InverseKinematicsMethod::LEVENBERG_MARQUARDT, "LEVENBERG_MARQUARDT"},
        {InverseKinematicsMethod::PARTICLE_SWARM, "PARTICLE_SWARM"},
        {InverseKinematicsMethod::BFGS, "BFGS"}};
    return method_names[method];
}

int main(int argc, char* argv[]) {
    // Load model
    const int n_joints = 20;
    auto link_5        = import_urdf<double, n_joints>("../data/urdfs/nugus.urdf");
    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = link_5.random_configuration();
    auto q0                                     = link_5.home_configuration();
    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "left_foot_base";
    std::string source_link_name = "torso";
    Hst_desired                  = forward_kinematics(link_5, q_random, target_link_name, source_link_name);
    // Compute the inverse kinematics for the random desired transform
    InverseKinematicsOptions<double, n_joints> options;
    options.max_iterations = 1000;
    options.tolerance      = 1e-5;

    std::vector<InverseKinematicsMethod> methods = {InverseKinematicsMethod::JACOBIAN,
                                                    InverseKinematicsMethod::NLOPT,
                                                    InverseKinematicsMethod::LEVENBERG_MARQUARDT,
                                                    InverseKinematicsMethod::PARTICLE_SWARM,
                                                    InverseKinematicsMethod::BFGS};

    for (const auto& method : methods) {
        options.method = method;
        auto start     = std::chrono::high_resolution_clock::now();
        Eigen::Matrix<double, n_joints, 1> q_solution =
            inverse_kinematics<double, n_joints>(link_5, target_link_name, source_link_name, Hst_desired, q0, options);
        auto stop     = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        auto Hst_solution = forward_kinematics(link_5, q_solution, target_link_name, source_link_name);
        std::cout << method_to_string(method) << " Method took " << duration.count() << " us, with error of "
                  << homogeneous_error(Hst_desired, Hst_solution).squaredNorm() << std::endl;
    }

    return 0;
}