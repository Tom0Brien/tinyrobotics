#include <iostream>

#include "../include/model.hpp"
#include "../include/parser.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Create a robot model with 5 joints
    auto model = importURDF<double, 5>("../data/urdfs/5_link.urdf");

    // Display details of model
    model.showDetails();

    // Generate a random configuration vector
    Eigen::Matrix<double, 5, 1> q = model.homeConfiguration();

    // Define the source and target links
    std::string source_link_name = "link_2";
    std::string target_link_name = "link_5";

    // Compute the forward kinematics to link_5 from the base link at a random configuration
    auto H = model.forwardKinematics(q, target_link_name);
    std::cout << "Transform from base -> link_5: \n" << H.matrix() << std::endl;

    // Compute the forward kinematics to the link_5 from link_2 at a random configuration
    auto H2 = model.forwardKinematics(q, target_link_name, source_link_name);
    std::cout << "Transform from link_2 -> link_5: \n" << H2.matrix() << std::endl;

    // Compute the geometric Jacobian to link_5 at the random configuration
    auto J = model.jacobian(q, target_link_name);
    std::cout << "J: \n" << J << std::endl;

    // Compute the geometric Jacobian to link_5's COM at the random configuration
    Eigen::Matrix<double, 6, 5> Jcom = model.jacobianCOM(q, target_link_name);
    std::cout << "Jcom: \n" << Jcom << std::endl;

    // Compute the centre of mass of the robot from the base link at the random configuration
    auto com = model.centreOfMass(q);
    std::cout << "CoM from base frame: \n" << com << std::endl;

    // Compute the centre of mass of the robot from link_2 at the random configuration
    auto com2 = model.centreOfMass(q, source_link_name);
    std::cout << "CoM from link_2 frame: \n" << com2 << std::endl;

    return EXIT_SUCCESS;
}
