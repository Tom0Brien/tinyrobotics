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
    std::string path_to_urdf = "../data/urdfs/panda_arm.urdf";
    auto robot_model         = RML::model_from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    robot_model.show_details();

    Eigen::Matrix<double, 7, 1> q0 = robot_model.home_configuration<7>();
    Eigen::Matrix<double, 7, 1> p0 = Eigen::Matrix<double, 7, 1>::Zero();
    // p0 << 0, 1, 0, 0;
    Eigen::Matrix<double, 7, 1> u0 = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 2, 1> tspan;
    tspan << 0.0, 10.0;
    double dt = 0.1;

    // Run solver
    std::vector<Eigen::Matrix<double, 14, 1>> x_history;
    x_history = RML::solver(robot_model, q0, p0, u0, tspan, dt, RML::IntegrationMethod::EULER());

    std::cout << "xk = \n" << x_history[50] << std::endl;

    // Save the results
    RML::save_history(robot_model, x_history);

    return EXIT_SUCCESS;
}
