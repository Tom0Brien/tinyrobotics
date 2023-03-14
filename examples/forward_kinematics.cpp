#include <iostream>

#include "../include/Kinematics.hpp"
#include "../include/Model.hpp"
#include "../include/Parser.hpp"

int main(int argc, char* argv[]) {
    // Create a robot model with 4 joints
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    const int n_joints       = 4;
    auto model               = tr::model_from_urdf<double, n_joints>(path_to_urdf);

    std::cout << "Model.base_link_idx: " << model.base_link_idx << std::endl;

    // Create a configuration vector of zeros
    auto q0 = model.home_configuration();

    // Compute the forward kinematics to the left foot at the home configuration
    auto H0 = tr::forward_kinematics(model, q0, "left_foot");

    std::cout << "Left foot transform home configuration: \n" << H0.matrix() << std::endl;

    return EXIT_SUCCESS;
}
