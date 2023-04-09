#define CATCH_INVERSEKINEMATICS
#include "../include/inversekinematics.hpp"

#include <chrono>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "../include/kinematics.hpp"
#include "../include/parser.hpp"
#include "catch2/catch.hpp"


using namespace std::chrono;
using namespace tinyrobotics;

TEST_CASE("Test inverse kinematics Kuka with perfect initial condition", "[InverseKinematics]") {
    auto kuka_model = import_urdf<double, 7>("data/urdfs/kuka.urdf");

    // Make a random configuration
    auto q_random = kuka_model.random_configuration();
    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "kuka_arm_7_link";
    std::string source_link_name = "calib_kuka_arm_base_link";
    Hst_desired                  = forward_kinematics(kuka_model, q_random, source_link_name, target_link_name);
    // Compute the inverse kinematics for the random desired transform
    auto q0 = q_random + 0.1 * kuka_model.random_configuration();
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, 7, 1> q_solution =
        inverse_kinematics<double, 7>(kuka_model, source_link_name, target_link_name, Hst_desired, q0);
    // Stop timer
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Duration microseconds: " << duration.count() << std::endl;
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
    REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-2);
    // REQUIRE(orientation_error < 1e-2);
}
