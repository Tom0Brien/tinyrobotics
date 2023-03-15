#define CATCH_KINEMATICS
#include <chrono>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <string>

#include "../include/InverseKinematics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Parser.hpp"
#include "catch2/catch.hpp"

using namespace ifopt;
using namespace std::chrono;
using namespace tr;
using namespace tr::parser;
using namespace tr::kinematics;
using namespace tr::ik;

// Load the robot model from a URDF file
auto robot_model = from_urdf<double, 4>("data/urdfs/simple.urdf");

TEST_CASE("Test forward kinematics with link names", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration();
    q << 1, 2, 3, 4;
    Eigen::Transform<double, 3, Eigen::Isometry> Hst;
    std::string source_link_idx = "ground";
    std::string target_link_idx = "left_foot";
    auto start                  = high_resolution_clock::now();
    Hst                         = forward_kinematics(robot_model, q, 0, 6);
    auto stop                   = high_resolution_clock::now();
    auto duration               = duration_cast<microseconds>(stop - start);
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
    auto start          = high_resolution_clock::now();
    Hst                 = forward_kinematics(robot_model, q, 0, 6);
    auto stop           = high_resolution_clock::now();
    auto duration       = duration_cast<microseconds>(stop - start);
    std::cout << "Forward Kinematics computation took " << duration.count() << " microseconds" << std::endl;
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
    Hstc                         = forward_kinematics_com(robot_model, q, source_link_name, "left_leg");
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
    auto start                    = high_resolution_clock::now();
    Eigen::Matrix<double, 6, 4> J = geometric_jacobian(robot_model, q, target_link_name);
    auto stop                     = high_resolution_clock::now();
    auto duration                 = duration_cast<microseconds>(stop - start);
    std::cout << "Geometric Jacobian computation took " << duration.count() << " microseconds" << std::endl;
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

    auto kuka_model = from_urdf<double, 7>("data/urdfs/kuka.urdf");
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

TEST_CASE("Test inverse kinematics simple with initial conditions close to solution", "[InverseKinematics]") {
    const int ITERATIONS = 25;
    auto nugus_model     = from_urdf<double, 20>("data/urdfs/nugus.urdf");
    double total_time    = 0;
    for (int i = 0; i < ITERATIONS; ++i) {
        // Make a random configuration
        auto q_random = nugus_model.home_configuration();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
        std::string target_link_name = "torso";
        std::string source_link_name = "left_hip_yaw";
        Hst_desired                  = forward_kinematics(nugus_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        auto q0    = q_random + 0.05 * nugus_model.random_configuration();
        auto start = high_resolution_clock::now();
        Eigen::Matrix<double, 20, 1> q_solution =
            inverse_kinematics<double, 20>(nugus_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop     = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        total_time += duration.count();
        // std::cout << "Duration [ms]: " << duration.count() << std::endl;
        // Compute the forward kinematics for the solution
        Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
        Hst_solution = forward_kinematics(nugus_model, q_solution, source_link_name, target_link_name);

        // Compute the euler angles for current
        Eigen::Matrix<double, 3, 3> Rst_solution = Hst_solution.linear();

        // Compute the euler angles for desired
        Eigen::Matrix<double, 3, 3> Rst_desired = Hst_desired.linear();

        Eigen::Matrix<double, 3, 3> R_error = Rst_desired * Rst_solution.transpose();
        double orientation_error            = (Eigen::Matrix<double, 3, 3>::Identity() - R_error).diagonal().sum();

        // // Check that the solution is close to the desired transform
        // std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
        // std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
        // std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() <<
        // std::endl;
        // std::cout << "orientation_error:  " << orientation_error << std::endl;
        // std::cout << "q_random: " << q_random.transpose() << std::endl;
        // std::cout << "q0: " << q0.transpose() << std::endl;
        // std::cout << "q_solution: " << q_solution.transpose() << std::endl;

        REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 1e-2);
    }
    std::cout << "Inverse Kinematics computation took " << total_time / ITERATIONS << " microseconds" << std::endl;
}

TEST_CASE("Test inverse kinematics Kuka", "[Kinematics]") {
    const int ITERATIONS = 25;
    auto kuka_model      = from_urdf<double, 7>("data/urdfs/kuka.urdf");

    for (int i = 0; i < ITERATIONS; ++i) {
        // Make a random configuration
        auto q_random = kuka_model.random_configuration();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
        std::string target_link_name = "kuka_arm_7_link";
        std::string source_link_name = "calib_kuka_arm_base_link";
        Hst_desired                  = forward_kinematics(kuka_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        auto q0    = q_random + 0.1 * kuka_model.random_configuration();
        auto start = high_resolution_clock::now();
        Eigen::Matrix<double, 7, 1> q_solution =
            inverse_kinematics<double, 7>(kuka_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop     = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        // std::cout << "Duration milliseconds: " << duration.count() << std::endl;
        // Compute the forward kinematics for the solution
        Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
        Hst_solution = forward_kinematics(kuka_model, q_solution, source_link_name, target_link_name);

        // Compute the euler angles for current
        Eigen::Matrix<double, 3, 3> Rst_solution = Hst_solution.linear();

        // Compute the euler angles for desired
        Eigen::Matrix<double, 3, 3> Rst_desired = Hst_desired.linear();

        Eigen::Matrix<double, 3, 3> R_error = Rst_desired * Rst_solution.transpose();
        double orientation_error            = (Eigen::Matrix<double, 3, 3>::Identity() - R_error).diagonal().sum();

        // Check that the solution is close to the desired transform
        // std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
        // std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
        // std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() <<
        // std::endl;
        // std::cout << "q_random: " << q_random.transpose() << std::endl;
        // std::cout << "q0: " << q0.transpose() << std::endl;
        // std::cout << "q_solution: " << q_solution.transpose() << std::endl;

        REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 1e-1);
    }
}
