#define CATCH_FMIN

#include "../include/fmin.hpp"

#include <iostream>

#include "catch2/catch.hpp"

using namespace tinyrobotics;

double rosenbrock(const std::vector<double>& x, std::vector<double>& grad, void* my_func_data) {
    double a     = 1.0;
    double b     = 100.0;
    double term1 = pow(a - x[0], 2);
    double term2 = b * pow(x[1] - pow(x[0], 2), 2);
    double fx    = term1 + term2;

    if (!grad.empty()) {
        grad[0] = -2.0 * (a - x[0]) - 4.0 * b * x[0] * (x[1] - pow(x[0], 2));
        grad[1] = 2.0 * b * (x[1] - pow(x[0], 2));
    }

    return fx;
}


TEST_CASE("Rosenbrock function optimization", "[nlopt]") {
    // Set up the optimizer
    nlopt::opt opt(nlopt::LD_MMA, 2);
    opt.set_min_objective(rosenbrock, nullptr);
    opt.set_xtol_rel(1e-12);

    // Set the initial guess
    std::vector<double> x(2);
    x[0] = -1.0;
    x[1] = 1.0;

    // Optimize the function
    double minf;
    nlopt::result result = opt.optimize(x, minf);

    // Check the results
    REQUIRE(x[0] == Approx(1.0).epsilon(1e-4));
    REQUIRE(x[1] == Approx(1.0).epsilon(1e-4));
}
