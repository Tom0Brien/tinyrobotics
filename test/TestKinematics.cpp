#define CATCH_KINEMATICS
#include <chrono>
#include <string>

#include "../include/Kinematics.hpp"
#include "../include/Parser.hpp"
#include "catch2/catch.hpp"

using namespace std::chrono;
using namespace tinyrobotics;

// Load the robot model from a URDF file
auto robot_model = import_urdf<double, 4>("data/urdfs/simple.urdf");

TEST_CASE("Test forward kinematics with link names", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Transform<double, 3, Eigen::Isometry> Hst;
    std::string source_link_idx = "ground";
    std::string target_link_idx = "left_foot";
    Hst                         = forward_kinematics(robot_model, q, 0, 6);
    // Check that the transform is correct
    Eigen::Matrix<double, 4, 4> Hst_expected;
    Hst_expected << -0.9900, 0, -0.1411, 1.1411, 0, 1.0000, 0, 0, 0.1411, 0, -0.9900, 2.9900, 0, 0, 0, 1.0000;
    REQUIRE(Hst.matrix().isApprox(Hst_expected, 1e-4));
};

TEST_CASE("Test forward kinematics with link idx", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Transform<double, 3, Eigen::Isometry> Hst;
    int target_link_idx = 0;
    int source_link_idx = 6;
    Hst                 = forward_kinematics(robot_model, q, 0, 6);
    // Check that the transform is correct
    Eigen::Matrix<double, 4, 4> Hst_expected;
    Hst_expected << -0.9900, 0, -0.1411, 1.1411, 0, 1.0000, 0, 0, 0.1411, 0, -0.9900, 2.9900, 0, 0, 0, 1.0000;
    REQUIRE(Hst.matrix().isApprox(Hst_expected, 1e-4));
};

TEST_CASE("Test forward kinematics to centre of mass", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Transform<double, 3, Eigen::Isometry> Hstc;
    std::string target_link_name = "left_leg";
    std::string source_link_name = "ground";
    Hstc                         = forward_kinematics_com(robot_model, q, source_link_name, target_link_name);
    // Check that the transform is correct
    Eigen::Matrix<double, 4, 4> Hstc_expected;
    Hstc_expected << -0.9900, 0, -0.1411, 1.0706, 0, 1, 0, 0, 0.1411, 0, -0.9900, 2.4950, 0, 0, 0, 1;
    REQUIRE(Hstc.matrix().isApprox(Hstc_expected, 1e-4));
}

TEST_CASE("Test geometric_jacobian calculations for simple model", "[ForwardKinematics]") {
    // Create a configuration for the robot
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    // Compute the geometric jacobian of robot with respect to the ground
    std::string target_link_name  = "left_foot";
    Eigen::Matrix<double, 6, 4> J = geometric_jacobian(robot_model, q, target_link_name);
    // Check that the geometric jacobian is correct
    Eigen::Matrix<double, 6, 4> J_expected;
    J_expected << 1.0, 0, -0.9899924966, 0, 0, 0, 0, 0, 0, 1.0, 0.14112, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0;
    REQUIRE(J.isApprox(J_expected, 1e-4));
    // Test for another target link
    target_link_name = "right_foot";
    J                = geometric_jacobian(robot_model, q, target_link_name);
    // Check that the geometric jacobian is correct
    J_expected << 1.0, 0, 0, -0.6536, 0, 0, 0, 0, 0, 1.0, 0, -0.7568, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0;
    REQUIRE(J.isApprox(J_expected, 1e-4));
}

TEST_CASE("Test geometric_jacobian_com calculations for simple model", "[ForwardKinematics]") {
    // Create a configuration for the robot
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    // Compute the geometric jacobian of robot with respect to the ground
    std::string target_link_name  = "left_foot";
    Eigen::Matrix<double, 6, 4> J = geometric_jacobian_com(robot_model, q, target_link_name);
    // Check that the geometric jacobian is correct
    Eigen::Matrix<double, 6, 4> J_expected;
    J_expected << 1.0, 0, -0.9899924966, 0, 0, 0, 0, 0, 0, 1.0, 0.14112, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0;
    REQUIRE(J.isApprox(J_expected, 1e-4));
}

TEST_CASE("Test geometric_jacobian calculations for kuka model", "[ForwardKinematics]") {

    auto kuka_model = import_urdf<double, 7>("data/urdfs/kuka.urdf");
    // Create a configuration for the robot
    auto q = kuka_model.home_configuration();
    q << 1, 2, 3, 4, 5, 6, 7;
    // Compute the geometric jacobian of robot with respect to the ground
    std::string target_link_name = "kuka_arm_7_link";
    auto J                       = geometric_jacobian(kuka_model, q, target_link_name);
    // Check that the geometric jacobian is correct
    Eigen::Matrix<double, 6, 7> J_expected;
    J_expected << 0.2175, -0.1519, -0.3056, -0.2059, -0.0038, 0.0767, 0, -0.1046, -0.2365, 0.1816, -0.3988, -0.0209,
        -0.0128, 0, -0.0000, -0.2395, 0.0269, 0.1212, 0.0049, 0.0058, 0, 0.0000, 0.8415, -0.4913, 0.8648, 0.2425,
        0.1763, -0.0337, 0.0000, -0.5403, -0.7651, -0.4855, 0.1801, 0.9581, 0.2352, 1.0000, -0.0000, -0.4161, -0.1283,
        0.9533, -0.2258, 0.9714;
    REQUIRE(J.isApprox(J_expected, 1e-4));
}

TEST_CASE("Test centre of mass", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Matrix<double, 3, 1> rCBb;
    std::string source_link_idx = "ground";
    std::string target_link_idx = "left_foot";
    rCBb                        = centre_of_mass(robot_model, q, std::string("ground"));
    // Check that the centre of mass is correct
    Eigen::Matrix<double, 3, 1> rCBb_expected;
    rCBb_expected << 923.0397e-003, 0.0000e+000, 2.2055e+000;
    REQUIRE(rCBb.isApprox(rCBb_expected, 1e-4));
};
