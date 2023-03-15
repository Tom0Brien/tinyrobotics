#define CATCH_DYNAMICS
#include <Eigen/Dense>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <chrono>
#include <string>

#include "../include/Dynamics.hpp"
#include "../include/Parser.hpp"
#include "catch2/catch.hpp"

using namespace tr::parser;
using namespace tr::dynamics;
using namespace tr::kinematics;


TEST_CASE("Test mass matrix for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration();
    // Compute the dynamics
    auto start = std::chrono::high_resolution_clock::now();
    mass_matrix(robot_model, q);

    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Mass Matrix computation took " << duration.count() << " microseconds" << std::endl;
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 4, 4> M_expected;
    M_expected << 20, 0, 2.5, 2.5, 0, 20, 0, 0, 2.5, 0, 1.25108, 0, 2.5, 0, 0, 1.25;

    REQUIRE(robot_model.data.M.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test mass matrix for kuka model", "[Dynamics]") {
    // Create a robot model
    auto kuka_model = from_urdf<double, 7>("data/urdfs/kuka.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 7, 1> q = kuka_model.home_configuration();
    // Compute the mass matrix
    mass_matrix(kuka_model, q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 7, 7> M_expected;
    M_expected << 0.0136, 0.0346, 0.0072, -0.0186, 0.0008, 0, 0, 0.0346, 4.4728, 0.0402, -1.9636, 0.0290, 0.1354, 0,
        0.0072, 0.0402, 0.0072, -0.0186, 0.0008, 0, 0, -0.0186, -1.9636, -0.0186, 0.9620, -0.0130, -0.0730, 0, 0.0008,
        0.0290, 0.0008, -0.0130, 0.0008, 0, 0, 0, 0.1354, 0, -0.0730, 0, 0.0122, 0, 0, 0, 0, 0, 0, 0, 0;
    REQUIRE(kuka_model.data.M.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test kinetic, potential and total energy computation for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = from_urdf<double, 4>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> p;
    p << 1, 2, 3, 4;
    // Compute the kinetic, potential and hamiltonian
    hamiltonian(robot_model, q, p);
    // Check that the kinetic, potential and hamiltonian are correct
    REQUIRE(robot_model.data.T - 83.1250 < 1e-2);
    REQUIRE(robot_model.data.V - 432.7102 < 1e-2);
    REQUIRE(robot_model.data.H - 515.8352 < 1e-2);
};

TEST_CASE("Test hamiltonian dynamics for simple model", "[Dynamics]") {
    // Create a robot model
    auto robot_model = from_urdf<double, 4>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> p;
    p << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> u = robot_model.home_configuration();
    u << 1, 2, 3, 4;
    // Compute the kinetic, potential and hamiltonian
    auto start = std::chrono::high_resolution_clock::now();
    forward_dynamics(robot_model, q, p, u);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Hamiltonian Dynamics computation took " << duration.count() << " microseconds"
              << std::endl;  // Check that the hamiltonian dynamics are correct
    Eigen::Matrix<double, 8, 1> dx_dt_expected;
    dx_dt_expected << 1.0111, 0.5284, 4.2528, 5.3216, 1.0000, -194.2000, -7.5397, 28.1456;
    REQUIRE(robot_model.data.dx_dt.isApprox(dx_dt_expected, 1e-4));
};

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 2 link model", "[Dynamics]") {
    const int n_joints = 2;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/2_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 3 link model", "[Dynamics]") {
    const int n_joints = 3;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/3_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 4 link model", "[Dynamics]") {
    const int n_joints = 4;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/4_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 5 link model", "[Dynamics]") {
    const int n_joints = 5;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/5_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}


TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for compass model", "[Dynamics]") {
    const int n_joints = 4;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/simple.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for panda_arm", "[Dynamics]") {
    const int n_joints = 7;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/panda_arm.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = forward_dynamics_ab(robot_model, q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for NUgus model", "[Dynamics]") {
    const int n_joints = 20;
    auto robot_model   = from_urdf<double, n_joints>("data/urdfs/nugus.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q     = Eigen::Matrix<double, n_joints, 1>::Ones();
    Eigen::Matrix<double, n_joints, 1> qd    = Eigen::Matrix<double, n_joints, 1>::Ones();
    q(0)                                     = 10;
    Eigen::Matrix<double, n_joints, 1> tau   = Eigen::Matrix<double, n_joints, 1>::Ones();
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    // Start the timer
    auto start                             = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, n_joints, 1> qdd = forward_dynamics_ab(robot_model, q, qd, tau);
    // Stop the timer
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Dynamics for 20dof robot via Articulated-Body Algorithm computation took " << duration.count()
              << " microseconds" << std::endl;
}
