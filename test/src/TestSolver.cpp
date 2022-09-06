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


TEST_CASE("Test single euler integration step for simple model", "[Solver]") {
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
    result = RML::euler_step(robot_model, q0, p0, u0, dt);
}

TEST_CASE("Test integration routine for simple model with euler integration", "[Solver]") {
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
    RML::SolverParams<double> params;
    params.dt                 = 0.1;
    params.tspan              = std::pair<float, float>(0.0, 10.0);
    params.integration_method = RML::IntegrationMethod::RK4;
    auto results              = RML::solver(robot_model, q0, p0, u0, params);

    // TODO: Verify results
}

TEST_CASE("Test integration routine for simple model with symplectic euler integration", "[Solver]") {
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
    RML::SolverParams<double> params;
    params.dt                 = 0.1;
    params.tspan              = std::pair<float, float>(0.0, 10.0);
    params.integration_method = RML::IntegrationMethod::SYMPLECTIC_EULER;
    auto results              = RML::solver(robot_model, q0, p0, u0, params);

    // TODO: Verify results
}