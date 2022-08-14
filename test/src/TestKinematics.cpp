#define CATCH_KINEMATICS
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <string>

#include "../../include/ForwardKinematics.hpp"
#include "../../include/InverseKinematics.hpp"
#include "../../include/UrdfParser.hpp"
#include "catch2/catch.hpp"

using namespace ifopt;
#include <chrono>
using namespace std::chrono;

// Load the robot model from a URDF file
auto robot_model = RML::model_from_urdf<double>("data/urdfs/simple.urdf");

TEST_CASE("Test forward kinematics", "[ForwardKinematics]") {
    auto start = high_resolution_clock::now();
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration<4>();
    q << 1, 2, 3, 4;

    Eigen::Transform<double, 3, Eigen::Affine> Hst;
    std::string target_link_name = "left_foot";
    std::string source_link_name = "ground";
    Hst                          = RML::forward_kinematics(robot_model, q, source_link_name, target_link_name);
    auto stop                    = high_resolution_clock::now();
    auto duration                = duration_cast<microseconds>(stop - start);
    std::cout << "FK time: " << duration.count() << "us" << std::endl;
    // Check that the transform is correct

    Eigen::Matrix<double, 4, 4> Hst_expected;
    Hst_expected << -0.9900, 0, -0.1411, 1.1411, 0, 1.0000, 0, 0, 0.1411, 0, -0.9900, 2.9900, 0, 0, 0, 1.0000;

    REQUIRE(Hst.matrix().isApprox(Hst_expected, 1e-4));
};

TEST_CASE("Test forward kinematics to centre of mass", "[ForwardKinematics]") {
    // Compute FK for a given configuration
    auto q = robot_model.home_configuration<4>();
    q << 1, 2, 3, 4;

    Eigen::Transform<double, 3, Eigen::Affine> Hstc;
    std::string target_link_name = "left_leg";
    std::string source_link_name = "ground";
    Hstc                         = RML::forward_kinematics_com(robot_model, q, source_link_name, target_link_name);

    // Check that the transform is correct
    Eigen::Matrix<double, 4, 4> Hstc_expected;
    Hstc_expected << -0.9900, 0, -0.1411, 1.0706, 0, 1, 0, 0, 0.1411, 0, -0.9900, 2.4950, 0, 0, 0, 1;

    REQUIRE(Hstc.matrix().isApprox(Hstc_expected, 1e-4));
}

// TEST_CASE("Test translation geometric jacobian", "[ForwardKinematics]") {
//     // Compute the translation geometric jacobian of the left foot wrt the ground
//     auto q = robot_model.home_configuration<4>();
//     q << 1, 2, 3, 4;
//     std::string target_link_name = "left_foot";
//     std::string source_link_name = "ground";
//     Eigen::MatrixXd Jv           = RML::Jv(robot_model, q, source_link_name, target_link_name);

//     // Check the jacobian is correct
//     Eigen::MatrixXd J_expected;
//     J_expected.resize(3, 4);
//     J_expected << 1.0, 0, -0.9899924966, 0, 0, 0, 0, 0, 0, 1.0, 0.14112, 0;
//     CHECK(Jv(0, 0) - J_expected(0, 0) < 1e-8);
//     CHECK(Jv(1, 0) - J_expected(1, 0) < 1e-8);
//     CHECK(Jv(2, 0) - J_expected(2, 0) < 1e-8);
//     CHECK(Jv(0, 1) - J_expected(0, 1) < 1e-8);
//     CHECK(Jv(1, 1) - J_expected(1, 1) < 1e-8);
//     CHECK(Jv(2, 1) - J_expected(2, 1) < 1e-8);
//     CHECK(Jv(0, 2) - J_expected(0, 2) < 1e-8);
//     CHECK(Jv(1, 2) - J_expected(1, 2) < 1e-8);
//     CHECK(Jv(2, 2) - J_expected(2, 2) < 1e-8);
//     CHECK(Jv(0, 3) - J_expected(0, 3) < 1e-8);
//     CHECK(Jv(1, 3) - J_expected(1, 3) < 1e-8);
//     CHECK(Jv(2, 3) - J_expected(2, 3) < 1e-8);
// };

// TEST_CASE("Test centre of mass calculations for simple model", "[ForwardKinematics]") {
//     // Create a configuration for the robot
//     auto q = robot_model.home_configuration<4>();
//     q << 1, 2, 3, 4;
//     // Compute the centre of mass of robot with respect to the ground
//     std::string source_link_name = "ground";
//     Eigen::Vector3d com          = RML::centre_of_mass(robot_model, q, source_link_name);
//     // Check that the centre of mass is correct
//     Eigen::Vector3d com_expected;
//     com_expected << 0.9230, 0, 2.2055;
//     REQUIRE(com.isApprox(com_expected, 1e-4));

//     // Create another configuration for the robot
//     q << 4, 3, 2, 1;
//     // Compute the centre of mass of robot with respect to the ground
//     com = RML::centre_of_mass(robot_model, q, source_link_name);
//     // Check that the centre of mass is correct
//     com_expected << 4.2188, 0, 2.9845;
//     REQUIRE(com.isApprox(com_expected, 1e-4));
// }

// TEST_CASE("Test geometric_jacobian calculations for simple model", "[ForwardKinematics]") {
//     // Create a configuration for the robot
//     auto q = robot_model.home_configuration<4>();
//     q << 1, 2, 3, 4;
//     // Compute the geometric jacobian of robot with respect to the ground
//     std::string target_link_name  = "left_foot";
//     Eigen::Matrix<double, 6, 4> J = RML::geometric_jacobian(robot_model, q, target_link_name);
//     // Check that the geometric jacobian is correct
//     Eigen::Matrix<double, 6, 4> J_expected;
//     J_expected.resize(6, 4);
//     J_expected << 1.0, 0, -0.9899924966, 0, 0, 0, 0, 0, 0, 1.0, 0.14112, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0;
//     REQUIRE(J.isApprox(J_expected, 1e-4));

//     // Test for another target link
//     target_link_name = "right_foot";
//     J                = RML::geometric_jacobian(robot_model, q, target_link_name);
//     // Check that the geometric jacobian is correct
//     J_expected << 1.0, 0, 0, -0.6536, 0, 0, 0, 0, 0, 1.0, 0, -0.7568, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0;
//     REQUIRE(J.isApprox(J_expected, 1e-4));
// }

// TEST_CASE("Test geometric_jacobian calculations for nugus model", "[ForwardKinematics]") {
//     // <RML::Model<double>> ngus_model = RML::Model<double>::from_urdf("data/urdfs/nugus.urdf");
//     // // Create a configuration for the robot
//     // Eigen::Matrix<double, 4, 1> q = nugus_model.home_configuration();
//     // nugus_model.show_details();
//     // // q << 1, 2, 3, 4,5 ,6 ,7 ,8,9,10,11,12,13,14,15,16,17,18,19,20;
//     // // Compute the geometric jacobian of robot with respect to the ground
//     // std::string target_link_name = "left_ankle";
//     // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J = RML::geometric_jacobian(nugus_model, q,
//     // target_link_name); std::cout << "J: " << std::endl << J << std::endl; Check that the geometric jacobian is
//     // correct Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_expected; J_expected.resize(6, 4); J_expected
//     // << 1.0, 0, -0.9899924966, 0,
//     //               0, 0, 0, 0,
//     //               0, 1.0, 0.14112, 0,
//     //               0, 0, 0, 0,
//     //               0, 0, -1.0, 0,
//     //               0, 0, 0, 0;
//     // REQUIRE(J.isApprox(J_expected, 1e-4));
// }


TEST_CASE("Test inverse kinematics simple with initial conditions close to solution", "[InverseKinematics]") {
    const int ITERATIONS = 25;
    auto nugus_model     = RML::model_from_urdf<double>("data/urdfs/nugus.urdf");
    nugus_model.show_details();
    double total_time = 0;
    for (int i = 0; i < ITERATIONS; ++i) {
        // Make a random configuration
        auto q_random = nugus_model.random_configuration<20>();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
        std::string target_link_name = "left_foot";
        std::string source_link_name = "left_hip_yaw";
        Hst_desired = RML::forward_kinematics(nugus_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        auto q0    = q_random + 0.05 * nugus_model.random_configuration<20>();
        auto start = high_resolution_clock::now();
        Eigen::Matrix<double, 20, 1> q_solution =
            RML::inverse_kinematics<double, 20>(nugus_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop     = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        total_time += duration.count();
        // std::cout << "Duration [ms]: " << duration.count() << std::endl;
        // Compute the forward kinematics for the solution
        Eigen::Transform<double, 3, Eigen::Affine> Hst_solution;
        Hst_solution = RML::forward_kinematics(nugus_model, q_solution, source_link_name, target_link_name);

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
        // std::cout << "orientation_error:  " << orientation_error << std::endl;
        // std::cout << "q_random: " << q_random.transpose() << std::endl;
        // std::cout << "q0: " << q0.transpose() << std::endl;
        // std::cout << "q_solution: " << q_solution.transpose() << std::endl;

        REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 1e-1);
    }
    std::cout << "Average duration [ms]: " << total_time / ITERATIONS << std::endl;
}

TEST_CASE("Test inverse kinematics Kuka", "[Kinematics]") {
    const int ITERATIONS = 25;
    auto kuka_model      = RML::model_from_urdf<double>("data/urdfs/kuka.urdf");

    for (int i = 0; i < ITERATIONS; ++i) {
        // Make a random configuration
        auto q_random = kuka_model.random_configuration<7>();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
        std::string target_link_name = "kuka_arm_7_link";
        std::string source_link_name = "calib_kuka_arm_base_link";
        Hst_desired = RML::forward_kinematics(kuka_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        auto q0    = q_random + 0.1 * kuka_model.random_configuration<7>();
        auto start = high_resolution_clock::now();
        Eigen::Matrix<double, 7, 1> q_solution =
            RML::inverse_kinematics<double, 7>(kuka_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop     = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        // std::cout << "Duration milliseconds: " << duration.count() << std::endl;
        // Compute the forward kinematics for the solution
        Eigen::Transform<double, 3, Eigen::Affine> Hst_solution;
        Hst_solution = RML::forward_kinematics(kuka_model, q_solution, source_link_name, target_link_name);

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
