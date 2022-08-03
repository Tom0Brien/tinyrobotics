#define CATCH_KINEMATICS
#include "catch2/catch.hpp"
#include <string>

#include "../../src/RobotModel.hpp"
#include "../../src/ForwardKinematics.hpp"
#include "../../src/InverseKinematics.hpp"

// Load the robot model from a URDF file
std::shared_ptr<RML::RobotModel<double>> robot_model = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

TEST_CASE("Test forward kinematics", "[Kinematics]"){
    // Compute FK for a given configuration
    Eigen::VectorXd q = robot_model->home_configuration();
    q << 1, 2, 3, 4;

    Eigen::Transform<double, 3, Eigen::Affine> Hst;
    std::string target_link_name = "left_foot";
    std::string source_link_name = "ground";
    Hst = RML::forward_kinematics(robot_model, q, source_link_name, target_link_name);

    // Check that the transform is correct

    Eigen::Matrix<double, 4, 4> Hst_expected;
    Hst_expected << -0.9900,  0,      -0.1411, 1.1411,
                    0,        1.0000, 0,       0,
                    0.1411,   0,      -0.9900, 2.9900,
                    0,        0,      0,       1.0000;

    REQUIRE(Hst.matrix().isApprox(Hst_expected, 1e-4));

};

TEST_CASE("Test forward kinematics to centre of mass", "[Kinematics]"){
    // Compute FK for a given configuration
    Eigen::VectorXd q = robot_model->home_configuration();
    q << 1, 2, 3, 4;

    Eigen::Transform<double, 3, Eigen::Affine> Hstc;
    std::string target_link_name = "left_leg";
    std::string source_link_name = "ground";
    Hstc = RML::forward_kinematics_com(robot_model, q, source_link_name, target_link_name);

    // Check that the transform is correct
    Eigen::Matrix<double, 4, 4> Hstc_expected;
    Hstc_expected << -0.9900, 0, -0.1411, 1.0706,
                    0, 1,       0,      0,
                    0.1411, 0, -0.9900, 2.4950,
                    0, 0,       0,      1;

    REQUIRE(Hstc.matrix().isApprox(Hstc_expected, 1e-4));

}

TEST_CASE ("Test translation geometric jacobian", "[Kinematics]") {
    // Compute the translation geometric jacobian of the left foot wrt the ground
    Eigen::VectorXd q = robot_model->home_configuration();
    q << 1, 2, 3, 4;
    std::string target_link_name = "left_foot";
    std::string source_link_name = "ground";
    Eigen::MatrixXd Jv = RML::Jv(robot_model, q, source_link_name, target_link_name);

    // Check the jacobian is correct
    Eigen::MatrixXd J_expected;
    J_expected.resize(3, 4);
    J_expected << 1.0, 0, -0.9899924966, 0,
                  0, 0, 0, 0,
                  0, 1.0, 0.14112, 0;
    CHECK(Jv(0,0) - J_expected(0,0) < 1e-8);
    CHECK(Jv(1,0) - J_expected(1,0) < 1e-8);
    CHECK(Jv(2,0) - J_expected(2,0) < 1e-8);
    CHECK(Jv(0,1) - J_expected(0,1) < 1e-8);
    CHECK(Jv(1,1) - J_expected(1,1) < 1e-8);
    CHECK(Jv(2,1) - J_expected(2,1) < 1e-8);
    CHECK(Jv(0,2) - J_expected(0,2) < 1e-8);
    CHECK(Jv(1,2) - J_expected(1,2) < 1e-8);
    CHECK(Jv(2,2) - J_expected(2,2) < 1e-8);
    CHECK(Jv(0,3) - J_expected(0,3) < 1e-8);
    CHECK(Jv(1,3) - J_expected(1,3) < 1e-8);
    CHECK(Jv(2,3) - J_expected(2,3) < 1e-8);
};

#include <chrono>
using namespace std::chrono;

TEST_CASE ("Test inverse kinematics simple", "[Kinematics]") {
    const int ITERATIONS = 1000;
    std::shared_ptr<RML::RobotModel<double>> nugus_model = RML::RobotModel<double>::from_urdf("data/urdfs/nugus.urdf");
    for(int i = 0; i < ITERATIONS; ++i){
        // Make a random configuration
        Eigen::VectorXd q_random = nugus_model->random_configuration();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
        std::string target_link_name = "left_foot";
        std::string source_link_name = "torso";
        Hst_desired = RML::forward_kinematics(nugus_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        Eigen::VectorXd q0 = nugus_model->home_configuration();
        auto start = high_resolution_clock::now();
        Eigen::VectorXd q_solution = RML::inverse_kinematics(nugus_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        std::cout << "Duration [ms]: " << duration.count() << std::endl;
        // Compute the forward kinematics for the solution
        Eigen::Transform<double, 3, Eigen::Affine> Hst_solution;
        Hst_solution = RML::forward_kinematics(nugus_model, q_solution, source_link_name, target_link_name);

        // Compute the euler angles for current
        Eigen::Matrix<double, 3, 3> Rst_solution = Hst_solution.linear();

        // Compute the euler angles for desired
        Eigen::Matrix<double, 3, 3> Rst_desired = Hst_desired.linear();

        Eigen::Matrix<double, 3, 3> R_error = Rst_desired * Rst_solution.transpose();
        double orientation_error = (Eigen::Matrix<double, 3, 3>::Identity() - R_error).diagonal().sum();
        // Check that the solution is close to the desired transform
        std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
        std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
        std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
        std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() << std::endl;

        // REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 5e-1);
    }
}

// TEST_CASE ("Test inverse kinematics Kuka", "[Kinematics]") {
//     const int ITERATIONS = 1000;
//     std::shared_ptr<RML::RobotModel<double>> kuka_model = RML::RobotModel<double>::from_urdf("data/urdfs/kuka.urdf");
//     kuka_model->show_details();

//     for(int i = 0; i < ITERATIONS; ++i){
//         // Make a random configuration
//         Eigen::VectorXd q_random = kuka_model->random_configuration();
//         // Compute the forward kinematics for the random configuration
//         Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
//         std::string target_link_name = "kuka_arm_7_link";
//         std::string source_link_name = "calib_kuka_arm_base_link";
//         Hst_desired = RML::forward_kinematics(kuka_model, q_random, source_link_name, target_link_name);
//         // Compute the inverse kinematics for the random desired transform
//         Eigen::VectorXd q0 = kuka_model->home_configuration();
//         auto start = high_resolution_clock::now();
//         Eigen::VectorXd q_solution = RML::inverse_kinematics(kuka_model, source_link_name, target_link_name, Hst_desired, q0);
//         auto stop = high_resolution_clock::now();
//         auto duration = duration_cast<milliseconds>(stop - start);
//         std::cout << "Duration microseconds: " << duration.count() << std::endl;
//         // Compute the forward kinematics for the solution
//         Eigen::Transform<double, 3, Eigen::Affine> Hst_solution;
//         Hst_solution = RML::forward_kinematics(kuka_model, q_solution, source_link_name, target_link_name);
//         // Check that the solution is close to the desired transform
//         std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
//         std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
//         std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
//         std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() << std::endl;

//         REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
//     }
// }
