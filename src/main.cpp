#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "matplotlibcpp.h"

using namespace autodiff;

#include "Model.hpp"


int main(int argc, char* argv[]) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::microseconds;
    using std::chrono::milliseconds;

    // Create a robot model
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    auto robot_model         = RML::from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    robot_model.show_details();

    // Create a random configuration
    Eigen::Matrix<double, 20, 1> q_random = robot_model.random_configuration<20>();
    // Plot the configuration
    // namespace plt = matplotlibcpp;
    // std::vector<double> y = RML::eigen_to_vec<double>(q_random);
    // plt::plot(y, y);
    // plt::show();

    return EXIT_SUCCESS;
}
