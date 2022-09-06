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
    q0 << 0, 1, 0, 0;
    Eigen::Matrix<double, 4, 1> p0;
    p0 << 0, 0, 4, 4;
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
    RML::SolverParams<double> params;
    params.dt                 = 0.001;
    params.tspan              = std::pair<float, float>(0.0, 10.0);
    params.integration_method = RML::IntegrationMethod::RK4;
    params.active_constraints.push_back("left_foot");
    // params.active_constraints.push_back("body");

    // Run solver
    auto results = RML::solver(model, q0, p0, u0, params);

    // // Save the results
    RML::save_history(model, results.x_history);

    // Plot the results
    RML::plot_results(results);

    return EXIT_SUCCESS;
}
