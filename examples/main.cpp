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
    std::string path_to_urdf = "../data/urdfs/cart_pole.urdf";
    auto robot_model         = RML::model_from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    robot_model.show_details();

    Eigen::Matrix<double, 2, 1> q0;
    q0 << 1, 2;
    Eigen::Matrix<double, 2, 1> p0;
    p0 << 3, 4;
    Eigen::Matrix<double, 2, 1> u0 = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 2, 1> tspan;
    tspan << 0.0, 10.0;
    double dt = 0.01;

    // Compute forward dynamics
    RML::hamiltonian_dynamics(robot_model, q0, p0, u0);
    std::cout << "Forward dynamics computed" << std::endl;
    std::cout << "dx_dt: " << std::endl << robot_model.results.dx_dt << std::endl;
    RML::mass_matrix(robot_model, q0);
    std::cout << "M: " << std::endl << robot_model.results.M << std::endl;
    std::cout << "Potential energy: " << robot_model.results.V << std::endl;

    // Run solver
    auto results = RML::solver(robot_model, q0, p0, u0, tspan, dt, RML::IntegrationMethod::RK4());

    std::cout << "xk = \n" << results.x_history[50] << std::endl;

    // Save the results
    RML::save_history(robot_model, results.x_history);

    // Plot the results
    RML::plot_results(results);

    return EXIT_SUCCESS;
}
