#include <iostream>

#include "../include/dynamics.hpp"
#include "../include/model.hpp"
#include "../include/parser.hpp"


using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Create a robot model with 5 joints
    auto model = import_urdf<double, 5>("../data/urdfs/5_link.urdf");

    // Display details of model
    model.show_details();

    // Define the source and target links
    const std::string source_link_name = "link_2";
    const std::string target_link_name = "link_5";

    // Set some random joint configuration, velocity and torque
    auto q   = model.random_configuration();
    auto qd  = model.random_configuration();
    auto tau = model.random_configuration();

    // Compute the forward dynamics
    auto qdd = forward_dynamics(model, q, qd, tau);
    std::cout << "qdd: \n" << qdd << std::endl;

    // Compute the inverse dynamics
    auto tau2 = inverse_dynamics(model, q, qd, qdd);
    std::cout << "tau: \n" << tau2 << std::endl;

    // Compute the mass matrix
    auto M = mass_matrix(model, q);
    std::cout << "Mass matrix: \n" << M << std::endl;

    // Compute the potential energy
    auto V = potential_energy(model, q);
    std::cout << "Potential energy: \n" << V << std::endl;

    // Compute the kinetic energy
    auto T = kinetic_energy(model, q, qd);
    std::cout << "Kinetic energy: \n" << T << std::endl;

    // Compute the total energy
    auto E = total_energy(model, q, qd);
    std::cout << "Total energy: \n" << E << std::endl;

    return EXIT_SUCCESS;
}
