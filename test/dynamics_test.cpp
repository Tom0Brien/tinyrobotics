#define CATCH_DYNAMICS

#include <Eigen/Dense>
#include <chrono>

#include "../include/parser.hpp"
#include "catch2/catch.hpp"

using namespace tinyrobotics;

TEST_CASE("Test mass matrix for 2 link model", "[Dynamics]") {
    // Create a robot model
    auto model = importURDF<double, 2>("data/urdfs/2_link.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 2, 1> q = model.homeConfiguration();
    // Compute the dynamics
    model.massMatrix(q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 2, 2> M_expected;
    M_expected << 2.6692, 0.8346, 0.8346, 0.3346;
    REQUIRE(model.mass_matrix.isApprox(M_expected, 1e-4));
};


TEST_CASE("Test mass matrix for 4 link model", "[Dynamics]") {
    // Create a robot model
    auto model = importURDF<double, 4>("data/urdfs/4_link.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = model.homeConfiguration();
    // Compute the dynamics
    model.massMatrix(q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 4, 4> M_expected;
    M_expected << 21.3384, 13.5038, 6.6692, 1.8346, 13.5038, 9.0038, 4.6692, 1.3346, 6.6692, 4.6692, 2.6692, 0.8346,
        1.8346, 1.3346, 0.8346, 0.3346;
    REQUIRE(model.mass_matrix.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test mass matrix for simple model", "[Dynamics]") {
    // Create a robot model
    auto model = importURDF<double, 4>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = model.homeConfiguration();
    // Compute the dynamics
    model.massMatrix(q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 4, 4> M_expected;
    M_expected << 20, 0, 2.5, 2.5, 0, 20, 0, 0, 2.5, 0, 1.25108, 0, 2.5, 0, 0, 1.25;
    REQUIRE(model.mass_matrix.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test mass matrix for kuka model", "[Dynamics]") {
    // Create a robot model
    auto model = importURDF<double, 7>("data/urdfs/kuka.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 7, 1> q = model.homeConfiguration();
    // Compute the mass matrix
    model.massMatrix(q);
    // Check that the mass matrix is correct
    Eigen::Matrix<double, 7, 7> M_expected;
    M_expected << 0.0136, 0.0346, 0.0072, -0.0186, 0.0008, 0, 0, 0.0346, 4.4728, 0.0402, -1.9636, 0.0290, 0.1354, 0,
        0.0072, 0.0402, 0.0072, -0.0186, 0.0008, 0, 0, -0.0186, -1.9636, -0.0186, 0.9620, -0.0130, -0.0730, 0, 0.0008,
        0.0290, 0.0008, -0.0130, 0.0008, 0, 0, 0, 0.1354, 0, -0.0730, 0, 0.0122, 0, 0, 0, 0, 0, 0, 0, 0;
    REQUIRE(model.mass_matrix.isApprox(M_expected, 1e-4));
};

TEST_CASE("Test kinetic, potential and total energy computation for simple model", "[Dynamics]") {
    // Create a robot model
    auto model = importURDF<double, 4>("data/urdfs/simple.urdf");
    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q = model.homeConfiguration();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> dq;
    dq << 1, 2, 3, 4;
    // Compute the kinetic, potential and totalEnergy
    model.totalEnergy(q, dq);
    // Check that the kinetic, potential and totalEnergy are correct
    REQUIRE(model.kinetic_energy - 83.1250 < 1e-2);
    REQUIRE(model.potential_energy - 432.7102 < 1e-2);
    REQUIRE(model.total_energy - 515.8352 < 1e-2);
};

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 2 link model", "[Dynamics]") {
    const int n_joints = 2;
    auto model         = importURDF<double, n_joints>("data/urdfs/2_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 3 link model", "[Dynamics]") {
    const int n_joints = 3;
    auto model         = importURDF<double, n_joints>("data/urdfs/3_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 4 link model", "[Dynamics]") {
    const int n_joints = 4;
    auto model         = importURDF<double, n_joints>("data/urdfs/4_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for 5 link model", "[Dynamics]") {
    const int n_joints = 5;
    auto model         = importURDF<double, n_joints>("data/urdfs/5_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Composite-Rigid-Body Algorithm for 5 link model", "[Dynamics]") {
    const int n_joints = 5;
    auto model         = importURDF<double, n_joints>("data/urdfs/5_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamicsCRB(q, qd, tau);
}


TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for compass model", "[Dynamics]") {
    const int n_joints = 4;
    auto model         = importURDF<double, n_joints>("data/urdfs/simple.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
    qdd                                      = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for panda_arm", "[Dynamics]") {
    const int n_joints = 7;
    auto model         = importURDF<double, n_joints>("data/urdfs/panda_arm.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> tau;
    tau << 1, 2, 3, 4, 5, 6, 7;
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Forward Dynamics via Articulated-Body Algorithm for NUgus model", "[Dynamics]") {
    const int n_joints = 20;
    auto model         = importURDF<double, n_joints>("data/urdfs/nugus.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q     = Eigen::Matrix<double, n_joints, 1>::Ones();
    Eigen::Matrix<double, n_joints, 1> qd    = Eigen::Matrix<double, n_joints, 1>::Ones();
    q(0)                                     = 10;
    Eigen::Matrix<double, n_joints, 1> tau   = Eigen::Matrix<double, n_joints, 1>::Ones();
    Eigen::Matrix<double, n_joints, 1> f_ext = Eigen::Matrix<double, n_joints, 1>::Zero();
    Eigen::Matrix<double, n_joints, 1> qdd   = model.forwardDynamics(q, qd, tau);
}

TEST_CASE("Test Inverse Dynamics for 5 link model", "[Dynamics]") {
    const int n_joints = 5;
    auto model         = importURDF<double, n_joints>("data/urdfs/5_link.urdf");
    // Create some inputs
    Eigen::Matrix<double, n_joints, 1> q;
    q << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> qd;
    qd << 1, 2, 3, 4, 5;
    Eigen::Matrix<double, n_joints, 1> qdd;
    qdd << 1, 2, 3, 4, 5;
    auto tau = model.inverseDynamics(q, qd, qdd);
}
