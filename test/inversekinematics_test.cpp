#define CATCH_INVERSEKINEMATICS
#include "../include/inversekinematics.hpp"

#include <chrono>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "../include/inversekinematics.hpp"
#include "../include/parser.hpp"
#include "catch2/catch.hpp"


using namespace std::chrono;
using namespace tinyrobotics;

double myvfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    (void) data;
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myvconstraint(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    my_constraint_data* d = reinterpret_cast<my_constraint_data*>(data);
    double a = d->a, b = d->b;
    if (!grad.empty()) {
        grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
        grad[1] = -1.0;
    }
    return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
}

double rosenbrock(const Eigen::Matrix<double, 2, 1>& x, Eigen::Matrix<double, 2, 1>& grad, void* /*data*/) {
    double fx = (1 - x(0)) * (1 - x(0)) + 100 * (x(1) - x(0) * x(0)) * (x(1) - x(0) * x(0));

    if (grad.size() > 0) {
        grad(0) = -2 * (1 - x(0)) - 400 * x(0) * (x(1) - x(0) * x(0));
        grad(1) = 200 * (x(1) - x(0) * x(0));
    }

    return fx;
}


TEST_CASE("Rosenbrock function optimization", "[nlopt]") {
    nlopt::opt opt("LD_MMA", 2);
    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL;
    lb[1] = 0;
    opt.set_lower_bounds(lb);
    opt.set_min_objective(myvfunc, NULL);
    my_constraint_data data[2] = {{2, 0}, {-1, 1}};
    opt.add_inequality_constraint(myvconstraint, &data[0], 1e-9);
    opt.add_inequality_constraint(myvconstraint, &data[1], 1e-9);
    opt.set_xtol_rel(1e-12);

    std::vector<double> x(2);
    x[0] = 1.234;
    x[1] = 5.678;
    double minf;

    // Optimize
    nlopt::result result = opt.optimize(x, minf);

    // Check the results
    REQUIRE(x[0] == Approx(0.3333333348).epsilon(1e-4));
    REQUIRE(x[1] == Approx(0.2962962891).epsilon(1e-4));
}

TEST_CASE("Rosenbrock function optimization with wrapper", "[nlopt]") {
    // Create the NLopt optimizer and set the algorithm
    nlopt::opt opt(nlopt::LN_COBYLA, 2);

    // Set the objective function
    ObjectiveFunction<double, 2> obj_fun = rosenbrock;
    opt.set_min_objective(eigen_objective_wrapper<double, 2>, &obj_fun);

    // Set the optimization tolerances
    opt.set_xtol_rel(1e-12);
    opt.set_ftol_rel(1e-12);

    // Set the initial guess
    Eigen::VectorXd x0(2);
    x0 << -1.2, 1.0;
    std::vector<double> x_initial(2);
    eigen_to_nlopt<double, 2>(x0, x_initial);

    // Optimize
    double minf;
    nlopt::result result = opt.optimize(x_initial, minf);

    // Convert the optimized solution back to an Eigen vector
    Eigen::Matrix<double, 2, 1> optimized_solution(2);
    nlopt_to_eigen<double, 2>(x_initial, optimized_solution);

    // Check that the optimization was successful and the result is close to the expected minimum
    REQUIRE(optimized_solution(0) == Approx(1.0).epsilon(1e-4));
    REQUIRE(optimized_solution(1) == Approx(1.0).epsilon(1e-4));
}

TEST_CASE("Rosenbrock function optimization with grad and wrapper", "[nlopt]") {
    // Set the initial guess
    const int nv = 2;
    Eigen::Matrix<double, nv, 1> x;
    x << -1.2, 1.0;

    // Set up the NLopt optimization problem with Rosenbrock as the objective function
    nlopt::opt opt(nlopt::LD_SLSQP, nv);
    ObjectiveFunction<double, nv> obj_fun = rosenbrock;
    opt.set_min_objective(eigen_objective_wrapper<double, nv>, static_cast<void*>(&obj_fun));
    opt.set_xtol_rel(1e-8);

    // Convert the initial guess to NLopt format
    std::vector<double> x0(nv);
    eigen_to_nlopt(x, x0);

    // Optimize the function using NLopt
    double minf;
    opt.optimize(x0, minf);
    nlopt_to_eigen(x0, x);

    // Check that the optimization was successful and the result is close to the expected minimum
    REQUIRE(x(0) == Approx(1.0).epsilon(1e-4));
    REQUIRE(x(1) == Approx(1.0).epsilon(1e-4));
}

TEST_CASE("Test inverse kinematics for 2-link robot with nlopt", "[inversekinematics]") {
    // Load model
    const int n_joints = 2;
    auto model         = importURDF<double, n_joints>("data/urdfs/2_link.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random;
    q_random << M_PI / 4, M_PI / 6;

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "end_effector";
    std::string source_link_name = "ground";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Compute the inverse kinematics for the random desired transform
    auto q0 = model.homeConfiguration();
    InverseKinematicsOptions<double, n_joints> options;
    options.method = InverseKinematicsMethod::NLOPT;
    Eigen::Matrix<double, n_joints, 1> q_solution =
        model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-3);
}

TEST_CASE("Test inverse kinematics for 2-link robot with nlopt autodiff", "[inversekinematics]") {
    // Load model
    const int n_joints = 2;
    auto model         = importURDF<double, n_joints>("data/urdfs/2_link.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random;
    q_random << M_PI / 4, M_PI / 6;

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "end_effector";
    std::string source_link_name = "ground";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Compute the inverse kinematics for the random desired transform
    auto q0 = model.homeConfiguration();
    InverseKinematicsOptions<double, n_joints> options;
    Eigen::Matrix<double, n_joints, 1> q_solution =
        model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    REQUIRE((Hst_desired.translation() - Hst_solution.translation()).squaredNorm() < 1e-3);
}

TEST_CASE("Test inverse kinematics for 2-link robot with jacobian method", "[inversekinematics]") {
    // Load model
    const int n_joints = 2;
    auto model         = importURDF<double, n_joints>("data/urdfs/2_link.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = model.randomConfiguration();

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "end_effector";
    std::string source_link_name = "ground";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Compute the inverse kinematics for the random desired transform
    auto q0 = model.homeConfiguration();
    InverseKinematicsOptions<double, n_joints> options;
    Eigen::Matrix<double, n_joints, 1> q_solution =
        model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    REQUIRE(homogeneousError(Hst_desired, Hst_solution).squaredNorm() < 1e-3);
}

TEST_CASE("Test inverse kinematics for 2-link robot with levenberg-marquardt method", "[inversekinematics]") {
    // Load model
    const int n_joints = 2;
    auto model         = importURDF<double, n_joints>("data/urdfs/2_link.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = model.randomConfiguration();

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "end_effector";
    std::string source_link_name = "ground";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Compute the inverse kinematics for the random desired transform
    auto q0 = model.homeConfiguration();
    InverseKinematicsOptions<double, n_joints> options;
    options.method = InverseKinematicsMethod::LEVENBERG_MARQUARDT;
    Eigen::Matrix<double, n_joints, 1> q_solution =
        model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    REQUIRE(homogeneousError(Hst_desired, Hst_solution).squaredNorm() < 1e-3);
}

TEST_CASE("Test inverse kinematics for model robot with nlopt optimisation method", "[inversekinematics]") {
    // Load model
    const int n_joints = 20;
    auto model         = importURDF<double, n_joints>("data/urdfs/nugus.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = model.randomConfiguration();

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "left_foot";
    std::string source_link_name = "torso";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Compute the inverse kinematics for the random desired transform
    auto q0 = model.homeConfiguration();
    InverseKinematicsOptions<double, n_joints> options;
    options.method = InverseKinematicsMethod::NLOPT;
    Eigen::Matrix<double, n_joints, 1> q_solution =
        model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);

    // Compute the forward kinematics for the solution
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_solution;
    Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);

    // Check that the solution is close to the desired transform
    REQUIRE(homogeneousError(Hst_desired, Hst_solution).squaredNorm() < 1e-3);
}

TEST_CASE("Test inverse kinematics for model robot with all IK methods", "[inversekinematics]") {
    // Load model
    const int n_joints = 20;
    auto model         = importURDF<double, n_joints>("data/urdfs/nugus.urdf");

    // Make a random configuration
    Eigen::Matrix<double, n_joints, 1> q_random = model.randomConfiguration();
    auto q0                                     = model.homeConfiguration();

    // Compute the forward kinematics for the random configuration
    Eigen::Transform<double, 3, Eigen::Isometry> Hst_desired;
    std::string target_link_name = "left_foot";
    std::string source_link_name = "torso";
    Hst_desired                  = model.forwardKinematics(q_random, target_link_name, source_link_name);

    // Set up options
    InverseKinematicsOptions<double, n_joints> options;
    options.max_iterations                       = 1000;
    options.tolerance                            = 1e-5;
    std::vector<InverseKinematicsMethod> methods = {InverseKinematicsMethod::JACOBIAN,
                                                    InverseKinematicsMethod::NLOPT,
                                                    InverseKinematicsMethod::LEVENBERG_MARQUARDT,
                                                    InverseKinematicsMethod::PARTICLE_SWARM,
                                                    InverseKinematicsMethod::BFGS};
    // Run IK for each method
    for (const auto& method : methods) {
        options.method = method;
        Eigen::Matrix<double, n_joints, 1> q_solution =
            model.inverseKinematics(target_link_name, source_link_name, Hst_desired, q0, options);
        auto Hst_solution = model.forwardKinematics(q_solution, target_link_name, source_link_name);
        REQUIRE(homogeneousError(Hst_desired, Hst_solution).squaredNorm() < 1e-3);
    }
}
