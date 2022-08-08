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
    std::shared_ptr<RML::Model<double, 4>> robot_model;

    // Load the robot model from a URDF file
    robot_model = RML::Model<double, 4>::from_urdf("../data/urdfs/simple.urdf");

    // Show details of the robot model
    robot_model->show_details();

    // Create a random configuration
    Eigen::VectorXd q_random = robot_model->random_configuration();
    // Plot the configuration
    // namespace plt = matplotlibcpp;
    // std::vector<double> y = RML::eigen_to_vec<double>(q_random);
    // plt::plot(y, y);
    // plt::show();

    return EXIT_SUCCESS;
}
