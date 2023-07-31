#include <iostream>

#include "../include/model.hpp"
#include "../include/parser.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Create a robot model with 5 joints
    auto model = importURDF<double, 5>("../data/urdfs/5_link.urdf");

    // Display details of model
    model.showDetails();

    // Define the source and target links
    const std::string source_link_name = "link_2";
    const std::string target_link_name = "link_5";

    // Set some random joint configuration, velocity and torque
    auto q   = model.randomConfiguration();
    auto qd  = model.randomConfiguration();
    auto tau = model.randomConfiguration();

    // Compute the forward dynamics
    auto qdd = model.forwardDynamics(q, qd, tau);
    std::cout << "qdd: \n" << qdd << std::endl;

    // Compute the inverse dynamics
    auto tau2 = model.inverseDynamics(q, qd, qdd);
    std::cout << "tau: \n" << tau2 << std::endl;

    // Compute the mass matrix
    auto M = model.massMatrix(q);
    std::cout << "Mass matrix: \n" << M << std::endl;

    // Compute the potential energy
    auto V = model.potentialEnergy(q);
    std::cout << "Potential energy: \n" << V << std::endl;

    // Compute the kinetic energy
    auto T = model.kineticEnergy(q, qd);
    std::cout << "Kinetic energy: \n" << T << std::endl;

    // Compute the total energy
    auto E = model.totalEnergy(q, qd);
    std::cout << "Total energy: \n" << E << std::endl;

    return EXIT_SUCCESS;
}
