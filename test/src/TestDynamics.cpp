#define CATCH_DYNAMICS
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <string>

#include "../../src/Dynamics.hpp"
#include "../../src/Model.hpp"
#include "catch2/catch.hpp"
using namespace autodiff;

TEST_CASE("Test dynamics", "[Dynamics]"){
    // Create a robot model
    // std::shared_ptr<RML::Model<double>> robot_model;

    // // Load the robot model from a URDF file
    // robot_model = RML::Model<double>::from_urdf("data/urdfs/simple.urdf");

    // // Compute FK for a given configuration
    // Eigen::VectorXd q = robot_model->home_configuration();
    // q << 1, 2, 3, 4;

    // // Compute the dynamics
    // RML::compute_dynamics(robot_model, q);
};
