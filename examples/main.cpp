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
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Solver.hpp"
#include "../include/UrdfParser.hpp"


/**
 * @brief Example event detection function
 * @details
 * @param Scalar The scalar type of the joint
 */
bool eventDetection(RML::Model<double, 4>& model, Eigen::Matrix<double, 4, 1>& q, Eigen::Matrix<double, 4, 1>& p) {
    // Get the position of the left foot
    std::string source_link_name = "world";
    std::string target_link_name = "right_foot";
    std::cout << "q: " << q.transpose() << std::endl;
    Eigen::Matrix<double, 3, 1> right_foot_pos = RML::position(model, q, source_link_name, target_link_name);
    // Rotate into ground frame
    Eigen::Matrix<double, 3, 3> Ry = RML::roty(-0.0524);
    std::cout << "Ry: " << Ry.transpose() << std::endl;
    Eigen::Matrix<double, 3, 1> right_foot_pos_ground = Ry * right_foot_pos;

    // Check if the left foot is below the ground
    std::cout << "foot height: " << right_foot_pos_ground.transpose() << std::endl;
    if (right_foot_pos_ground(2) <= 0 && right_foot_pos_ground(0) >= 0.1) {
        std::cout << "Right foot is below the ground" << std::endl;
        std::cout << "Right foot position: " << right_foot_pos_ground.transpose() << std::endl;
        return true;
    }
    else {
        return false;
    }
}

int main(int argc, char* argv[]) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::microseconds;
    using std::chrono::milliseconds;

    // Create a robot model
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    auto model               = RML::model_from_urdf<double, 4>(path_to_urdf);

    // Show details of the robot model
    model.show_details();

    Eigen::Matrix<double, 4, 1> q0;
    q0 << -0.2170, 0.9762, 0.2187, -0.3234;

    Eigen::Matrix<double, 4, 1> p0;
    p0 << 0, 0, -16.9340, 1.8667;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();

    // Compute hamiltonian with active constraints
    std::vector<std::string> active_constraints;
    active_constraints.push_back("left_foot");

    // Compute forward dynamics with active constraints
    auto start                       = high_resolution_clock::now();
    Eigen::Matrix<double, 8, 1> dxdt = RML::forward_dynamics(model, q0, p0, u0, active_constraints);
    std::cout << "dxdt: " << std::endl << dxdt << std::endl;
    auto end   = high_resolution_clock::now();
    auto total = duration_cast<microseconds>(end - start);
    std::cout << "Time: " << total.count() << " microseconds" << std::endl;
    // Setup solver
    RML::SolverParams<double, 4> params;
    params.dt                 = 0.001;
    params.tspan              = std::pair<float, float>(0.0, 10.0);
    params.integration_method = RML::IntegrationMethod::RK4;
    params.active_constraints.push_back("left_foot");
    params.event_detection   = &eventDetection;
    params.event_is_terminal = true;
    // params.active_constraints.push_back("body");

    // Run solver
    auto results = RML::solver(model, q0, p0, u0, params);

    // // Save the results
    RML::save_history(model, results.x_history);

    // Plot the results
    RML::plot_results(results);

    return EXIT_SUCCESS;
}
