#include <iostream>

#include "../include/Kinematics.hpp"
#include "../include/Model.hpp"
#include "../include/Parser.hpp"

using namespace tr;

int main(int argc, char* argv[]) {
    // Create a robot model with 4 joints
    auto model = import_urdf<double, 4>("../data/urdfs/4_link.urdf");

    // Display details of model
    model.show_details();

    // Generate a random configuration vector
    auto q = model.random_configuration();

    // Compute the forward kinematics to the left foot at the home configuration
    auto H = forward_kinematics(model, q, std::string("link_4"));
    std::cout << "H: \n" << H.matrix() << std::endl;

    return EXIT_SUCCESS;
}
