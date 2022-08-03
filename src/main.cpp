#include <cstdlib>
#include <cassert>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
using namespace autodiff;

#include "RobotModel.hpp"
#include "ForwardKinematics.hpp"


int main(int argc, char* argv [])
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;

    // Create a robot model
    std::shared_ptr<RML::RobotModel<double>> robot_model;

    // Load the robot model from a URDF file
    robot_model = RML::RobotModel<double>::from_urdf("../data/urdfs/simple.urdf");

    // Show details of the robot model
    robot_model->show_details();

    return EXIT_SUCCESS;
}
