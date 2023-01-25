#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
// #include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "../include/Dynamics.hpp"
#include "../include/InverseKinematics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Solver.hpp"
#include "../include/UrdfParser.hpp"


std::string swing_foot   = "right_foot";
std::string planted_foot = "left_foot";
std::string world_frame  = "world";

int main(int argc, char* argv[]) {
    // Load in NUgus Model
    std::string path_to_urdf = "../data/urdfs/robot.urdf";
    auto model               = RML::model_from_urdf<double, 20>(path_to_urdf);
    // Show details of the robot model
    model.show_details();

    // Set intial conditions for the robot
    auto q0 = model.home_configuration();

    // Compute the forward kinematics to the left foot
    auto source_frame = std::string("torso");
    auto target_frame = std::string("left_foot");
    auto Htl_0        = RML::forward_kinematics(model, q0, target_frame);

    std::cout << "Left foot transform at initial conditions: " << Htl_0.matrix() << std::endl;

    // Compute the inverse kinematics for moving the left foot upwards
    Eigen::Matrix<double, 3, 1> target_position;
    auto Htl_target = Htl_0;
    auto q          = q0;
    std::vector<Eigen::Matrix<double, 20, 1>> q_history;
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 50; i++) {
        // Add 0.01 to the x and z position
        Htl_target(0, 3) += 0.01;
        Htl_target(2, 3) += 0.01;
        // Compute the inverse kinematics
        q = RML::inverse_kinematics(model, source_frame, target_frame, Htl_target, q);
        q_history.push_back(q);

        // Print out the current configuration
        auto Htl = RML::forward_kinematics(model, q, target_frame);
    }
    // End timer
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
    std::cout << "Time taken by function: " << duration << " microseconds" << std::endl;

    nlohmann::json json_q_history;
    // Loop through the simulation history and save the q values with joint names
    for (int i = 0; i < q_history.size(); i++) {
        for (auto joint : model.joints) {
            if (joint.q_idx != -1) {
                json_q_history[std::to_string(i)][joint.name] = {q_history[i](joint.q_idx)};
            }
        }
    }
    // Write the json result to a file
    std::ofstream o("ik_results.json");
    o << std::setw(4) << json_q_history << std::endl;


    return EXIT_SUCCESS;
}
