#include <Eigen/Dense>
#include <chrono>
#include <string>

#include "../include/parser.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {

    // Parse URDF
    const int n_joints      = 5;
    auto model              = importURDF<double, n_joints>("../data/urdfs/5_link.urdf");
    std::string source_link = "ground";
    std::string target_link = "end_effector";

    // ************ Model Details ************
    model.showDetails();

    // ************ Forward Kinematics ************
    auto start    = std::chrono::high_resolution_clock::now();
    auto q        = model.randomConfiguration();
    auto H        = model.forwardKinematics(q, target_link);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Kinematics: " << duration.count() << " microseconds" << std::endl;

    // ************ Inverse Kinematics ************
    start = std::chrono::high_resolution_clock::now();
    InverseKinematicsOptions<double, n_joints> options;
    options.max_iterations = 1000;
    options.tolerance      = 1e-4;
    options.method         = InverseKinematicsMethod::LEVENBERG_MARQUARDT;
    auto q0                = model.homeConfiguration();
    auto q_sol             = model.inverseKinematics(target_link, source_link, H, q0, options);
    stop                   = std::chrono::high_resolution_clock::now();
    duration               = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Inverse Kinematics: " << duration.count() << " microseconds" << std::endl;

    // ************ Geometric Jacobian ************
    start    = std::chrono::high_resolution_clock::now();
    auto J   = model.jacobian(q, target_link);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Geometric Jacobian: " << duration.count() << " microseconds" << std::endl;

    // ************ Forward Dynamics ************
    start    = std::chrono::high_resolution_clock::now();
    auto qdd = model.forwardDynamics(q, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Dynamics ABA: " << duration.count() << " microseconds" << std::endl;

    start    = std::chrono::high_resolution_clock::now();
    qdd      = model.forwardDynamicsCRB(q, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Dynamics CRB: " << duration.count() << " microseconds" << std::endl;

    // ************ Inverse Dynamics ************
    start    = std::chrono::high_resolution_clock::now();
    auto tau = model.inverseDynamics(q, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Inverse Dynamics: " << duration.count() << " microseconds" << std::endl;

    // ************ Mass Matrix ************
    start    = std::chrono::high_resolution_clock::now();
    auto M   = model.massMatrix(q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Mass Matrix: " << duration.count() << " microseconds" << std::endl;

    // ************ Kinetic Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto T   = model.kineticEnergy(q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Kinetic Energy: " << duration.count() << " microseconds" << std::endl;

    // ************ Potential Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto V   = model.potentialEnergy(q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Potential Energy: " << duration.count() << " microseconds" << std::endl;

    // ************ Total Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto E   = model.totalEnergy(q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Total Energy: " << duration.count() << " microseconds" << std::endl;
}