#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
// #include <filesystem>
#include <iostream>
#include <string>

#include "../include/Model.hpp"
#include "../include/UrdfParser.hpp"

int main(int argc, char* argv[]) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::microseconds;
    using std::chrono::milliseconds;

    // Create a robot model
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    auto robot_model         = RML::model_from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    robot_model.show_details();

    // Create a random configuration
    Eigen::Matrix<double, 20, 1> q_random = robot_model.random_configuration<20>();

    return EXIT_SUCCESS;
}
