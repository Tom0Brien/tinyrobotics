#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <chrono>
#include <cstdlib>
// #include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "../include/Dynamics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Solver.hpp"
#include "../include/UrdfParser.hpp"

int main(int argc, char* argv[]) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::microseconds;
    using std::chrono::milliseconds;

    // Create a robot model
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    auto model               = RML::model_from_urdf<double>(path_to_urdf);

    // Show details of the robot model
    model.show_details();

    Eigen::Matrix<double, 4, 1> q0;
    q0 << 1, 2, 3, 4;
    Eigen::Matrix<double, 4, 1> p0;
    p0 << 0, 0, 1, 2;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 2, 1> tspan;
    tspan << 0.0, 10.0;
    double dt = 0.01;

    // // Compute forward dynamics
    // RML::mass_matrix(model, q0);
    // std::cout << "M: " << std::endl << model.results.M << std::endl;
    // std::cout << "Potential energy: " << model.results.V << std::endl;

    // // Compute constraint

    // // Start timer
    // auto start = high_resolution_clock::now();

    // std::string contact_point       = "left_foot";
    // Eigen::Matrix<double, 3, 4> Jc1 = RML::Jv(model, q0, contact_point);
    // contact_point                   = "body";
    // Eigen::Matrix<double, 3, 4> Jc2 = RML::Jv(model, q0, contact_point);
    // // Stack Jc1 and Jc2
    // Eigen::Matrix<double, 6, 4> Jc;
    // Jc << Jc1, Jc2;
    // std::cout << "Jc: " << std::endl << Jc << std::endl;
    // // Compute the null space matrix of Gc.'
    // Eigen::MatrixXd Jcp = RML::null<double>(Jc);
    // std::cout << "Jcp: " << std::endl << Jcp << std::endl;
    // std::cout << "Jc * Jcp:" << std::endl << Jc * Jcp << std::endl;
    // std::cout << "Gp: " << std::endl << model.results.Gp << std::endl;

    // auto Gr = Jcp.transpose() * model.results.Gp;
    // std::cout << "Gr: " << std::endl << Gr << std::endl;

    // auto Dr = Jcp.transpose() * model.results.Dp * Jcp;
    // std::cout << "Dr: " << std::endl << Dr << std::endl;

    // auto Cr = Eigen::Matrix<double, 2, 2>::Zero();
    // std::cout << "Cr: " << std::endl << Cr << std::endl;

    // auto Mr = Jcp.transpose() * model.results.M * Jcp;
    // std::cout << "Mr: " << std::endl << Mr << std::endl;

    // // End timer
    // auto end   = high_resolution_clock::now();
    // auto total = duration_cast<microseconds>(end - start);
    // std::cout << "Time: " << total.count() << " microseconds" << std::endl;


    // Compute hamiltonian with active constraints
    std::vector<std::string> active_constraints;
    active_constraints.push_back("left_foot");
    RML::hamiltonian2(model, q0, p0, active_constraints);

    // Compute forward dynamics with active constraints
    Eigen::Matrix<double, 8, 1> dxdt = RML::forward_dynamics(model, q0, p0, u0, active_constraints);

    // // Run solver
    // auto results = RML::solver(model, q0, p0, u0, tspan, dt, RML::IntegrationMethod::RK4());

    // std::cout << "xk = \n" << results.x_history[50] << std::endl;

    // // Save the results
    // RML::save_history(model, results.x_history);

    // Plot the results
    // RML::plot_results(results);

    return EXIT_SUCCESS;
}
