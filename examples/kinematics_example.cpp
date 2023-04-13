#include <iostream>

#include "../include/kinematics.hpp"
#include "../include/model.hpp"
#include "../include/parser.hpp"

using namespace tinyrobotics;

int main(int argc, char* argv[]) {
    // Create a robot model with 5 joints
    auto model = import_urdf<double, 5>("../data/urdfs/5_link.urdf");

    // Display details of model
    model.show_details();

    // Generate a random configuration vector
    auto q = model.home_configuration();

    // Define the source and target links
    const std::string source_link_name = "link_2";
    const std::string target_link_name = "link_5";

    // Compute the forward kinematics to link_5 from the base link at a random configuration
    auto H = forward_kinematics(model, q, target_link_name);
    std::cout << "Transform from base -> link_5: \n" << H.matrix() << std::endl;

    // Compute the forward kinematics to the link_5 from link_2 at a random configuration
    auto H2 = forward_kinematics(model, q, target_link_name, source_link_name);
    std::cout << "Transform from link_2 -> link_5: \n" << H2.matrix() << std::endl;

    // Compute just the translation of the link_5 from the base link at a random configuration
    auto p = translation(model, q, target_link_name);
    std::cout << "Translation from base -> link_5: \n" << p << std::endl;

    // Compute just the translation of the link_5 from link_2 at a random configuration
    auto p2 = translation(model, q, target_link_name, source_link_name);
    std::cout << "Translation from link_2 -> link_5: \n" << p2 << std::endl;

    // Compute just the rotation of the link_5 from the base link at a random configuration
    auto R = rotation(model, q, target_link_name);
    std::cout << "Rotation from base -> link_5: \n" << R.matrix() << std::endl;

    // Compute just the rotation of the link_5 from link_2 at a random configuration
    auto R2 = rotation(model, q, target_link_name, source_link_name);
    std::cout << "Rotation from link_2 -> link_5: \n" << R2.matrix() << std::endl;

    // Compute the geometric Jacobian to link_5 at the random configuration
    auto J = geometric_jacobian(model, q, target_link_name);
    std::cout << "J: \n" << J << std::endl;

    // Compute the geometric Jacobian to link_5's COM at the random configuration
    auto Jcom = geometric_jacobian_com(model, q, target_link_name);
    std::cout << "Jcom: \n" << Jcom << std::endl;

    // Compute the centre of mass of the robot from the base link at the random configuration
    auto com = centre_of_mass(model, q);
    std::cout << "CoM from base frame: \n" << com << std::endl;

    // Compute the centre of mass of the robot from link_2 at the random configuration
    auto com2 = centre_of_mass(model, q, source_link_name);
    std::cout << "CoM from link_2 frame: \n" << com2 << std::endl;

    return EXIT_SUCCESS;
}
