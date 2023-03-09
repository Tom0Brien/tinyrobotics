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
#include "../include/InverseKinematics.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Math.hpp"
#include "../include/Model.hpp"
#include "../include/Solver.hpp"
#include "../include/UrdfParser.hpp"


std::string swing_foot   = "right_foot";
std::string planted_foot = "left_foot";
std::string world_frame  = "world";

/**
 * @brief Example event detection function
 * @details
 * @param Scalar The scalar type of the joint
 */
bool eventDetection(RML::Model<double, 4>& model, Eigen::Matrix<double, 4, 1>& q, Eigen::Matrix<double, 4, 1>& p) {
    // Get the position of the left foot
    Eigen::Matrix<double, 3, 1> right_foot_pos = RML::position(model, q, world_frame, swing_foot);
    // Rotate into ground frame around y axis by -3 degrees
    Eigen::Matrix<double, 3, 3> Ry =
        Eigen::AngleAxisd(-3.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix<double, 3, 1> right_foot_pos_ground = Ry * right_foot_pos;

    Eigen::Matrix<double, 3, 1> swing_from_planted = RML::position(model, q, planted_foot, swing_foot);

    // Check if the left foot is below the ground
    if (right_foot_pos_ground(2) <= 0 && swing_from_planted(0) >= 0.1) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief Compute impact mapping between contact scenarios.
 * @param q_minus pre-impact joint configuration.
 * @param p_plus pre-impact momentum.
 * @param Ja pre-impact contact jacobian.
 * @param Jb post-impact contact jacobian.
 * @return The post impact state.
 */

Eigen::Matrix<double, 8, 1> impact_mapping(RML::Model<double, 4>& model,
                                           const Eigen::Matrix<double, 4, 1>& q_minus,
                                           const Eigen::Matrix<double, 4, 1>& p_minus,
                                           const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Ja,
                                           const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Jb) {
    // Compute the null space of Ja
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jap = RML::null(Ja);
    // Compute the null space of Jb
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jbp = RML::null(Jb);
    // Compute the mass matrix
    mass_matrix<double, 4>(model, q_minus);
    // Compute the impact mapping matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> QTMQ    = (Jap.transpose() * model.data.M * Jap);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> QTMQinv = QTMQ.inverse();

    const int nr = QTMQinv.rows();
    auto p_plus  = Jbp.transpose() * (model.data.M * Jap * QTMQinv) * p_minus.tail(nr);

    // Store the result
    Eigen::Matrix<double, 4 + 4, 1> x_plus;
    x_plus.head(4)  = q_minus;
    x_plus.tail(nr) = p_plus;
    return x_plus;
}

int main(int argc, char* argv[]) {
    // Create a robot model
    std::string path_to_urdf = "../data/urdfs/simple.urdf";
    auto model               = RML::model_from_urdf<double, 4>(path_to_urdf);

    // Show details of the robot model
    model.show_details();

    // Setup solver
    RML::SolverParams<double, 4> params;
    params.dt                 = 0.001;
    params.tspan              = std::pair<float, float>(0.0, 10.0);
    params.integration_method = RML::IntegrationMethod::RK4;
    params.active_constraints.push_back("left_foot");
    params.event_detection   = &eventDetection;
    params.event_is_terminal = true;

    // Run solver
    Eigen::Matrix<double, 4, 1> q0;
    q0 << -0.2170, 0.9762, 0.2187, -0.3234;

    Eigen::Matrix<double, 4, 1> p0;
    p0 << 0, 0, -16.9340, 1.8667;
    Eigen::Matrix<double, 4, 1> u0 = Eigen::Matrix<double, 4, 1>::Zero();
    auto results                   = RML::solver(model, q0, p0, u0, params);


    // Apply impact mapping
    Eigen::Matrix<double, 4, 1> q_minus = results.x_history.back().head(4);
    Eigen::Matrix<double, 4, 1> p_minus = results.x_history.back().tail(4);
    auto Ja                             = RML::Jv(model, q_minus, std::string("left_foot"));
    auto Jb                             = RML::Jv(model, q_minus, std::string("right_foot"));
    auto x_plus                         = impact_mapping(model, q_minus, p_minus, Ja, Jb);
    q0                                  = x_plus.head(4);
    p0                                  = x_plus.tail(4);
    swing_foot                          = "left_foot";
    planted_foot                        = "right_foot";
    params.active_constraints.clear();
    params.active_constraints.push_back("right_foot");
    auto results2 = RML::solver(model, q0, p0, u0, params);

    return EXIT_SUCCESS;
}
