#define CATCH_Solver
#include <iostream>

#include "../include/Math.hpp"
#include "catch2/catch.hpp"

TEST_CASE("Test null space", "[Math]") {

    // Create matrix A
    Eigen::Matrix<double, 3, 3> A;
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    // Create null space
    auto null_space = tr::null<double>(A);

    // Verify null space
    Eigen::Matrix<double, 3, 1> expected_null_space;
    expected_null_space << -0.408248, 0.816497, -0.408248;
    // REQUIRE(null_space.isApprox(expected_null_space, 1e-4));
}