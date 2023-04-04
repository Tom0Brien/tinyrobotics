#include <chrono>

#include "../include/Dynamics.hpp"
#include "../include/InverseKinematics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Parser.hpp"
#include "../include/Solver.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Load in Model from URDF
    const int n_joints = 20;
    auto model         = import_urdf<double, n_joints>("../data/urdfs/robot.urdf");

    // Set initial conditions for the robot
    auto q0 = model.home_configuration();

    // Compute the forward kinematics to the left foot
    auto source_frame = std::string("torso");
    auto target_frame = std::string("left_foot");
    auto Htl_0        = forward_kinematics(model, q0, target_frame);

    // Compute the inverse kinematics for moving the left foot upwards
    auto Htl_target = Htl_0;
    auto q          = q0;
    std::vector<Eigen::Matrix<double, n_joints, 1>> q_history;


    double total_translation_error = 0;
    double total_rotation_error    = 0;
    double total_time              = 0;
    const int n_iterations         = 10;
    for (int i = 0; i < n_iterations; i++) {
        // Move the target position
        Htl_target(0, 3) += 0.05;
        Htl_target(2, 3) += 0.05;

        // Compute the inverse kinematics
        // Start timer
        auto start = std::chrono::high_resolution_clock::now();
        q          = inverse_kinematics(model, source_frame, target_frame, Htl_target, q);
        // Stop timer
        auto stop     = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        total_time += duration.count();

        // Print out the current configuration
        auto Htl = forward_kinematics(model, q, target_frame);

        // Compute the error
        auto translation_error = (Htl_target.translation() - Htl.translation()).norm();
        auto rotation_error =
            (Eigen::Matrix<double, 3, 3>::Identity() - Htl_target.rotation() * Htl.rotation().transpose())
                .diagonal()
                .sum();
        total_translation_error += translation_error;
        total_rotation_error += rotation_error;
    }

    std::cout << "Average time taken by IK: " << total_time / n_iterations << " milliseconds" << std::endl;
    std::cout << "Average translation error: " << total_translation_error / 10 << std::endl;
    std::cout << "Average rotation error: " << total_rotation_error / 10 << std::endl;

    return EXIT_SUCCESS;
}
