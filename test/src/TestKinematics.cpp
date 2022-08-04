#define CATCH_KINEMATICS
#include "catch2/catch.hpp"
#include <string>

#include "../../src/RobotModel.hpp"
#include "../../src/ForwardKinematics.hpp"
#include "../../src/InverseKinematics.hpp"

// Load the robot model from a URDF file
std::shared_ptr<RML::RobotModel<double>> robot_model = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

TEST_CASE("Test forward kinematics", "[ForwardKinematics]"){
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

TEST_CASE("Test forward kinematics to centre of mass", "[ForwardKinematics]"){
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

TEST_CASE ("Test translation geometric jacobian", "[ForwardKinematics]") {
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

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include "test_vars_constr_cost.h"

using namespace ifopt;
#include <chrono>
using namespace std::chrono;

TEST_CASE ("Test autodiff jacobian", "[ForwardKinematics]") {
    // 1. define the problem
    Problem nlp;
    nlp.AddVariableSet  (std::make_shared<ExVariables>());
    nlp.AddConstraintSet(std::make_shared<ExConstraint>());
    nlp.AddCostSet      (std::make_shared<ExCost>());

    // 2. choose solver and options
    IpoptSolver ipopt;
    ipopt.SetOption("linear_solver", "ma57");
    ipopt.SetOption("jacobian_approximation", "exact");
    ipopt.SetOption("print_level", 0);
    // 3 . solve
    auto start = high_resolution_clock::now();
    ipopt.Solve(nlp);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    // std::cout << "Duration [ms]: " << duration.count() << std::endl;
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    // std::cout << x.transpose() << std::endl;

    // 4. test if solution correct
    double eps = 1e-5; //double precision
    assert(1.0-eps < x(0) && x(0) < 1.0+eps);
    assert(0.0-eps < x(1) && x(1) < 0.0+eps);
}

TEST_CASE("Test centre of mass calculations for simple model", "[ForwardKinematics]") {
    // Create a configuration for the robot
    Eigen::VectorXd q = robot_model->home_configuration();
    q << 1, 2, 3, 4;
    // Compute the centre of mass of robot with respect to the ground
    std::string source_link_name = "ground";
    Eigen::Vector3d com = RML::centre_of_mass(robot_model, q, source_link_name);
    // Check that the centre of mass is correct
    Eigen::Vector3d com_expected;
    com_expected << 0.9230, 0, 2.2055;
    REQUIRE(com.isApprox(com_expected, 1e-4));

    // Create another configuration for the robot
    q << 4, 3, 2, 1;
    // Compute the centre of mass of robot with respect to the ground
    com = RML::centre_of_mass(robot_model, q, source_link_name);
    // Check that the centre of mass is correct
    com_expected << 4.2188, 0, 2.9845;
    REQUIRE(com.isApprox(com_expected, 1e-4));
}



TEST_CASE ("Test inverse kinematics simple with initial conditions close to solution", "[InverseKinematics]") {
    const int ITERATIONS = 25;
    std::shared_ptr<RML::RobotModel<double>> nugus_model = RML::RobotModel<double>::from_urdf("data/urdfs/nugus.urdf");
    double total_time = 0;
    for(int i = 0; i < ITERATIONS; ++i){
        // Make a random configuration
        Eigen::VectorXd q_random = nugus_model->random_configuration();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
        std::string target_link_name = "left_foot";
        std::string source_link_name = "torso";
        Hst_desired = RML::forward_kinematics(nugus_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        Eigen::VectorXd q0 = q_random + 0.1*nugus_model->random_configuration();
        auto start = high_resolution_clock::now();
        Eigen::VectorXd q_solution = RML::inverse_kinematics(nugus_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop = high_resolution_clock::now();
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
        double orientation_error = (Eigen::Matrix<double, 3, 3>::Identity() - R_error).diagonal().sum();

        // Check that the solution is close to the desired transform
        // std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
        // std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
        // std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "q_random: " << q_random.transpose() << std::endl;
        // std::cout << "q0: " << q0.transpose() << std::endl;
        // std::cout << "q_solution: " << q_solution.transpose() << std::endl;

        REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 1e-1);
    }
    // std::cout << "Average duration [ms]: " << total_time/ITERATIONS << std::endl;
}

TEST_CASE ("Test inverse kinematics Kuka", "[Kinematics]") {
    const int ITERATIONS = 25;
    std::shared_ptr<RML::RobotModel<double>> kuka_model = RML::RobotModel<double>::from_urdf("data/urdfs/kuka.urdf");

    for(int i = 0; i < ITERATIONS; ++i){
        // Make a random configuration
        Eigen::VectorXd q_random = kuka_model->random_configuration();
        // Compute the forward kinematics for the random configuration
        Eigen::Transform<double, 3, Eigen::Affine> Hst_desired;
        std::string target_link_name = "kuka_arm_7_link";
        std::string source_link_name = "calib_kuka_arm_base_link";
        Hst_desired = RML::forward_kinematics(kuka_model, q_random, source_link_name, target_link_name);
        // Compute the inverse kinematics for the random desired transform
        Eigen::VectorXd q0 = q_random + 0.1*kuka_model->random_configuration();
        auto start = high_resolution_clock::now();
        Eigen::VectorXd q_solution = RML::inverse_kinematics(kuka_model, source_link_name, target_link_name, Hst_desired, q0);
        auto stop = high_resolution_clock::now();
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
        double orientation_error = (Eigen::Matrix<double, 3, 3>::Identity() - R_error).diagonal().sum();

        // Check that the solution is close to the desired transform
        // std::cout << "Hst_desired.translation(): " << Hst_desired.translation().transpose() << std::endl;
        // std::cout << "Hst_solution.translation(): " << Hst_solution.translation().transpose() << std::endl;
        // std::cout << "Hst_desired [r,p,y]: " << Hst_desired.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "Hst_solution [r,p,y]: " << Hst_solution.linear().eulerAngles(0,1,2).transpose() << std::endl;
        // std::cout << "q_random: " << q_random.transpose() << std::endl;
        // std::cout << "q0: " << q0.transpose() << std::endl;
        // std::cout << "q_solution: " << q_solution.transpose() << std::endl;

        REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
        REQUIRE(orientation_error < 1e-1);
    }
}



