#include <Eigen/Dense>
#include <chrono>
#include <string>

#include "../include/Dynamics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Parser.hpp"
#include "../include/Solver.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Load model
    const int n_joints      = 5;
    auto model              = import_urdf<double, n_joints>("../data/urdfs/5_link.urdf");
    std::string source_link = "ground";
    std::string target_link = "link_5";

    // ************ Model Details ************
    model.show_details();

    // ************ Forward Kinematics ************
    auto start    = std::chrono::high_resolution_clock::now();
    auto q        = model.random_configuration();
    auto H        = forward_kinematics(model, q, target_link);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Kinematics time: " << duration.count() << " microseconds" << std::endl;

    // ************ Geometric Jacobian ************
    start    = std::chrono::high_resolution_clock::now();
    auto J   = geometric_jacobian(model, q, target_link);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Geometric Jacobian time: " << duration.count() << " microseconds" << std::endl;

    // ************ Dynamics ************
    start    = std::chrono::high_resolution_clock::now();
    auto qdd = forward_dynamics(model, q, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Forward Dynamics time: " << duration.count() << " microseconds" << std::endl;

    // ************ Mass Matrix ************
    start    = std::chrono::high_resolution_clock::now();
    auto M   = mass_matrix(model, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Mass Matrix time: " << duration.count() << " microseconds" << std::endl;

    // ************ Kinetic Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto T   = kinetic_energy(model, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Kinetic Energy time: " << duration.count() << " microseconds" << std::endl;

    // ************ Potential Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto V   = potential_energy(model, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Potential Energy time: " << duration.count() << " microseconds" << std::endl;

    // ************ Total Energy ************
    start    = std::chrono::high_resolution_clock::now();
    auto E   = total_energy(model, q, q);
    stop     = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Total Energy time: " << duration.count() << " microseconds" << std::endl;
}