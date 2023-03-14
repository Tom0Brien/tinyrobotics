#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "../include/Dynamics.hpp"
#include "../include/InverseKinematics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Parser.hpp"
#include "../include/Solver.hpp"


std::string swing_foot   = "right_foot";
std::string planted_foot = "left_foot";
std::string world_frame  = "world";

int main(int argc, char* argv[]) {
    // Load in Model from URDF
    std::string path_to_urdf = "../data/urdfs/robot.urdf";
    const int n_joints       = 20;
    auto model               = tr::model_from_urdf<double, n_joints>(path_to_urdf);

    // Set initial conditions for the robot
    auto q0 = model.home_configuration();

    // Compute the forward kinematics to the left foot
    auto source_frame = std::string("torso");
    auto target_frame = std::string("left_foot");
    auto Htl_0        = tr::forward_kinematics(model, q0, target_frame);

    // Compute the inverse kinematics for moving the left foot upwards
    Eigen::Matrix<double, 3, 1> target_position;
    auto Htl_target = Htl_0;
    auto q          = q0;
    std::vector<Eigen::Matrix<double, 20, 1>> q_history;

    // Start timer
    auto start                     = std::chrono::high_resolution_clock::now();
    double total_translation_error = 0;
    double total_rotation_error    = 0;
    for (int i = 0; i < 10; i++) {
        // Add 0.01m to the x and z position
        Htl_target(0, 3) += 0.01;
        Htl_target(2, 3) += 0.01;

        // Compute the inverse kinematics
        q = tr::inverse_kinematics(model, source_frame, target_frame, Htl_target, q);
        q_history.push_back(q);

        // Print out the current configuration
        auto Htl = tr::forward_kinematics(model, q, target_frame);

        // Compute the error
        Eigen::Quaterniond quaternion_current(Htl.rotation());
        Eigen::Quaterniond quaternion_target(Htl_target.rotation());
        auto translation_error = (Htl_target.translation() - Htl.translation()).norm();
        auto rotation_error    = tr::quaternion_error(quaternion_current, quaternion_target);
        total_translation_error += translation_error;
        total_rotation_error += rotation_error;
        std::cout << "(Translation error, Rotation error): (" << translation_error << ", " << rotation_error << ")"
                  << " at iteration " << i << std::endl;
    }
    // Stop timer
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Total time taken by IK for 10 iterations: " << duration.count() << " milliseconds" << std::endl;
    std::cout << "Average translation error: " << total_translation_error / 10 << std::endl;
    std::cout << "Average rotation error: " << total_rotation_error / 10 << std::endl;

    return EXIT_SUCCESS;
}
