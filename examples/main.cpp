#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
// #include <filesystem>
#include <iostream>
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
    std::string path_to_urdf = "../data/urdfs/test_compass_gait_floating_hip.urdf";
    auto robot_model         = RML::model_from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    robot_model.show_details();

    // Create a random configuration
    Eigen::Matrix<double, 8, 1> q0 = robot_model.home_configuration<8>();
    // Compute the mass matrix
    RML::mass_matrix(robot_model, q0);

    // Print mass matrix
    std::cout << "Mass Matrix:" << std::endl;
    std::cout << robot_model.results.M << std::endl;

    Eigen::Matrix<double, 8, 1> p0 = Eigen::Matrix<double, 8, 1>::Zero();
    Eigen::Matrix<double, 8, 1> u0 = Eigen::Matrix<double, 8, 1>::Zero();
    double dt                      = 0.1;

    // Run a single step of euler integration
    Eigen::Matrix<double, 8, 1> result;
    RML::hamiltonian_dynamics(robot_model, q0, p0, u0);

    // // Print the result
    std::cout << "dxdt = \n" << robot_model.results.dx_dt << std::endl;


    return EXIT_SUCCESS;
}
