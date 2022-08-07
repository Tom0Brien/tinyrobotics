#include <cstdlib>
#include <cassert>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include "matplotlibcpp.h"

using namespace autodiff;

#include "Model.hpp"


int main(int argc, char* argv [])
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;

    // Create a robot model
    std::shared_ptr<RML::Model<double>> robot_model;

    // Load the robot model from a URDF file
    robot_model = RML::Model<double>::from_urdf("../data/urdfs/simple.urdf");

    // Show details of the robot model
    robot_model->show_details();

    // Create a random configuration
    Eigen::VectorXd q_random = robot_model->random_configuration();
    // Plot the configuration
    namespace plt = matplotlibcpp;
    std::vector<double> y = RML::eigen_to_vec<double>(q_random);
    plt::plot(y, y);
    plt::show();

    return EXIT_SUCCESS;
}
