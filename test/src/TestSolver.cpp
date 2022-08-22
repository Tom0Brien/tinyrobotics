#define CATCH_Solver
#include <Eigen/Dense>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <chrono>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "../../include/Solver.hpp"
#include "../../include/UrdfParser.hpp"
#include "catch2/catch.hpp"


TEST_CASE("Test single euler integration step for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

    // Create initial conditions
    Eigen::Matrix<double, 4, 1> q0 = robot_model.home_configuration<4>();
    Eigen::Matrix<double, 4, 1> p0 = Eigen::Matrix<double, 4, 1>::Zero();
    p0 << 0, 1, 0, 0;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();
    double dt                      = 0.1;

    // Run a single step of euler integration
    Eigen::Matrix<double, 8, 1> result;
    result = RML::integration_step(robot_model, q0, p0, u0, dt, RML::IntegrationMethod::EULER());

    // Print the result
    std::cout << "xk = \n" << result << std::endl;
}

TEST_CASE("Test integration routine for simple model with euler integration", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

    // Create initial conditions
    Eigen::Matrix<double, 4, 1> q0 = robot_model.home_configuration<4>();
    Eigen::Matrix<double, 4, 1> p0 = Eigen::Matrix<double, 4, 1>::Zero();
    p0 << 0, 1, 0, 0;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 2, 1> tspan;
    tspan << 0.0, 10.0;
    double dt = 0.1;

    // Run solver
    std::vector<Eigen::Matrix<double, 8, 1>> x_history;
    x_history = RML::solver(robot_model, q0, p0, u0, tspan, dt, RML::IntegrationMethod::EULER());

    // Print the result
    // std::cout << "xk = \n" << x_history[0] << std::endl;
    // std::cout << "xk = \n" << x_history[50] << std::endl;
    // std::cout << "xk = \n" << x_history[100] << std::endl;
}

TEST_CASE("Test integration routine for simple model with symplectic euler integration", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

    // Create initial conditions
    Eigen::Matrix<double, 4, 1> q0 = robot_model.home_configuration<4>();
    Eigen::Matrix<double, 4, 1> p0 = Eigen::Matrix<double, 4, 1>::Zero();
    p0 << 0, 1, 0, 0;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 2, 1> tspan;
    tspan << 0.0, 10.0;
    double dt = 0.01;

    // Run solver
    std::vector<Eigen::Matrix<double, 8, 1>> x_history;
    x_history = RML::solver(robot_model, q0, p0, u0, tspan, dt, RML::IntegrationMethod::SYMPLECTIC_EULER());

    // Print the result
    // std::cout << "xk = \n" << x_history[0] << std::endl;
    // std::cout << "xk = \n" << x_history[50] << std::endl;
    // std::cout << "xk = \n" << x_history[1000] << std::endl;
}