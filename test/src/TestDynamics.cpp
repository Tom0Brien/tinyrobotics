#define CATCH_DYNAMICS
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <string>

#include "../../src/Dynamics.hpp"
#include "../../src/Model.hpp"
#include "catch2/catch.hpp"
using namespace autodiff;

TEST_CASE("Test dynamics", "[Dynamics]") {
    // Create a robot model
    auto robot_model = RML::from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Create a random configuration
    Eigen::Matrix<double, 4, 1> q_random;
    q_random << 1, 2, 3, 4;

    // Compute the dynamics
    RML::compute_dynamics(robot_model, q_random);
};
