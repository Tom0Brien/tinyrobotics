#define CATCH_DYNAMICS
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <chrono>
#include <string>

#include "../../src/Dynamics.hpp"
#include "../../src/Model.hpp"
#include "catch2/catch.hpp"
using namespace autodiff;

TEST_CASE("Test dynamics", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::from_urdf<double>("data/urdfs/simple.urdf");

    // Create a random configuration
    auto q_random = robot_model.home_configuration<4>();
    // Compute the dynamics
    auto start                    = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, 4, 4> M = Eigen::Matrix<double, 4, 4>::Zero();
    Eigen::Matrix<double, 4, 4> C = Eigen::Matrix<double, 4, 4>::Zero();
    Eigen::Matrix<double, 4, 1> g = Eigen::Matrix<double, 4, 1>::Zero();
    double V;
    RML::compute_dynamics(robot_model, q_random, M, C, g, V);

    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Dynamics computation took " << duration.count() << " microseconds" << std::endl;
};
