#define CATCH_DYNAMICS
#include <Eigen/Dense>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <chrono>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "../../include/Dynamics.hpp"
#include "../../include/UrdfParser.hpp"
#include "catch2/catch.hpp"
using namespace autodiff;

// TEST_CASE("Test dynamics", "[Dynamics]") {
//     // Create a robot model
//     auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

//     // Create a random configuration
//     auto q_random = robot_model.home_configuration<4>();
//     // Compute the dynamics
//     auto start                    = std::chrono::high_resolution_clock::now();
//     Eigen::Matrix<double, 4, 4> M = Eigen::Matrix<double, 4, 4>::Zero();
//     Eigen::Matrix<double, 4, 4> C = Eigen::Matrix<double, 4, 4>::Zero();
//     Eigen::Matrix<double, 4, 1> g = Eigen::Matrix<double, 4, 1>::Zero();
//     double V;
//     RML::compute_dynamics(robot_model, q_random, M, C, g, V);

//     auto stop     = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//     std::cout << "Dynamics computation took " << duration.count() << " microseconds" << std::endl;
// };

TEST_CASE("Test mass matrix for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration<4>();
    // Compute the dynamics
    auto start = std::chrono::high_resolution_clock::now();
    RML::mass_matrix(robot_model, q);

    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Mass Matrix computation took " << duration.count() << " microseconds" << std::endl;
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 4, 4> M_expected;
    M_expected << 20, 0, 2.5, 2.5, 0, 20, 0, 0, 2.5, 0, 1.25108, 0, 2.5, 0, 0, 1.25;

    REQUIRE(robot_model.results.M.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test mass matrix for kuka model", "[Dynamics]") {
    // Create a robot model
    auto kuka_model = RML::model_from_urdf<double>("data/urdfs/kuka.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 7, 1> q = kuka_model.home_configuration<7>();
    // Compute the mass matrix
    RML::mass_matrix(kuka_model, q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 7, 7> M_expected;
    M_expected << 0.0136, 0.0346, 0.0072, -0.0186, 0.0008, 0, 0, 0.0346, 4.4728, 0.0402, -1.9636, 0.0290, 0.1354, 0,
        0.0072, 0.0402, 0.0072, -0.0186, 0.0008, 0, 0, -0.0186, -1.9636, -0.0186, 0.9620, -0.0130, -0.0730, 0, 0.0008,
        0.0290, 0.0008, -0.0130, 0.0008, 0, 0, 0, 0.1354, 0, -0.0730, 0, 0.0122, 0, 0, 0, 0, 0, 0, 0, 0;
    REQUIRE(kuka_model.results.M.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test kinetic, potential and total energy computation for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration<4>();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> dq;
    dq << 1, 2, 3, 4;
    // Compute the kinetic, potential and hamiltonian
    RML::total_energy(robot_model, q, dq);
    // Check that the kinetic, potential and hamiltonian are correct
    REQUIRE(robot_model.results.T - 83.1250 < 1e-2);
    REQUIRE(robot_model.results.V - 432.7102 < 1e-2);
    REQUIRE(robot_model.results.H - 515.8352 < 1e-2);
};

TEST_CASE("Test hamiltonian dynamics for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration<4>();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> p;
    p << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> u = robot_model.home_configuration<4>();
    u << 1, 2, 3, 4;
    // Compute the kinetic, potential and hamiltonian
    auto start = std::chrono::high_resolution_clock::now();
    RML::hamiltonian_dynamics(robot_model, q, p, u);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Hamiltonian Dynamics computation took " << duration.count() << " microseconds"
              << std::endl;  // Check that the hamiltonian dynamics are correct
    Eigen::Matrix<double, 8, 1> dx_dt_expected;
    dx_dt_expected << 1.0111, 0.5284, 4.2528, 5.3216, 1.0000, -194.2000, -7.5397, 28.1456;
    REQUIRE(robot_model.results.dx_dt.isApprox(dx_dt_expected, 1e-4));
};