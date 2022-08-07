#ifndef RML_DYNAMICS_HPP
#define RML_DYNAMICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <map>

#include "Model.hpp"
#include "ForwardKinematics.hpp"

#include "autodiff/forward/real.hpp"
#include "autodiff/forward/real/eigen.hpp"
using namespace autodiff;

namespace RML {

    // template <typename Scalar>
    // struct Dynamics {

    //     /// @brief The mass matrix of the model
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> mass_matrix;

    //     /// @brief The coriolis matrix of the robot model.
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> coriolis_matrix;

    //     /// @brief The potential energy V
    //     Scalar V = 0.0;

    //     /// @brief The kinetic energy T
    //     Scalar T = 0.0;

    //     /// @brief The hamiltonian H
    //     Scalar H = V + T;

    //     /// @brief The gravity torque vector of the robot model.
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> gravity_torque;

    //     /// @brief The holonomic constraints of the system
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc;

    // };

    // /**
    //  * @brief Construct a new Model object from URDF file description.
    //  * @param model The robot model.
    //  * @param q The joint configuration of the robot.
    //  */
    // template <typename Scalar>
    // void compute_dynamics(std::shared_ptr<Model<Scalar>> model, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q) {
    //     // Create Dynamics object for the model
    //     std::shared_ptr<Dynamics<Scalar>> dynamics = std::make_shared<Dynamics<Scalar>>();

    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> M;
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J;
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc;

    //     // Cast q to autodiff::real
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> q_real(q);
    //     // Cast model to autodiff::real
    //     std::shared_ptr<Model<autodiff::real>> model_real = model->template cast<autodiff::real>();
    //     // Create over parametrised system
    //     for (auto link = model_real->links.begin(); link != model_real->links.end(); link++) {
    //         // Compute FK to centre of mass
    //         Eigen::Transform<autodiff::real, 3, Eigen::Affine> Hbm = forward_kinematics_com(model_real, q_real, model_real->base_link->name, link->second->name);
    //         Eigen::Matrix<autodiff::real, 3, 1> rMBb = Hbm.translation();
    //         // Add links contribution to potential energy m*g*h
    //         dynamics->V = dynamics->V - link->second->mass * model->gravity.transpose() * rMBb.cast<Scalar>();
    //         if(link->second->joint->type == RML::JointType::REVOLUTE) {
    //             // Add to mass matrix
    //             // M.conservativeResizeLike(Eigen::Matrix<Scalar, M.rows() + 3, M.rows() + 3>::Zero());
    //             // M.block(M.rows() - 3, M.cols() - 3, 3, 3) = link->second->mass * Eigen::Matrix<Scalar, 3, 3>::Identity();
    //             // Add inertia to J matrix TODO: Probably need to actually load inertia information
    //             // J.resize(J.rows() + 1, J.cols() + 1);
    //             // J.block(J.rows() - 1, J.cols() - 1, 1, 1) = Eigen::Matrix<Scalar, 3, 3>::Zero();
    //         } else if (link->second->joint->type == RML::JointType::FIXED) {
    //             // Add to mass matrix
    //             // M.resize(M.rows() + 3, M.cols() + 3);
    //             // M.block(M.rows() - 3, M.cols() - 3, 3, 3) = link->second->mass * Eigen::Matrix<Scalar, 3, 3>::Identity();
    //             // // Add inertia to J matrix
    //             // J.resize(J.rows() + 1, J.cols() + 1);
    //             // J.block(J.rows() - 1, J.cols() - 1, 1, 1) = Eigen::Matrix<Scalar, 3, 3>::Zero(); // TODO: Probably need to actually load inertia information
    //             // // Add constraint to holonomic constraints
    //             // fc.resize(fc.rows() + 3);
    //             // fc.block(fc.rows() - 3, 1, 3, 1) = rMBb;
    //         } else if (link->second->joint->type == RML::JointType::PRISMATIC){
    //             // Add inertia to J matrix
    //             // J.resize(J.rows() + 1, J.cols() + 1);
    //             // J.block(J.rows() - 1, J.cols() - 1, 1, 1) = Eigen::Matrix<Scalar, 3, 3>::Zero(); // TODO: Probably need to actually load inertia information
    //         }

    //         // Print M
    //         // std::cout << "M: " << M << std::endl;
    //     }
    //     // Compute reduced system

    // };

    // /**
    //  * @brief Constraint function for holonomic constraints.
    //  * @param q The joint configuration of the robot.
    //  * @return The constraint function for holonomic constraints.
    //  */
    // VectorXreal fc_function(const VectorXreal& q) {
    //     return fc;
    // }

    // /**
    //  * @brief Eliminates holonomic constraints from the dynamic equations of motion via the appropriate selection
    //  * of generalised coordinates.
    //  * @param model The robot model.
    //  * @param q_real The joint configuration of the robot.
    //  * @param fc The holonomic constraints.
    //  * @return The mass matrix of the robot model.
    //  */
    // holonomic_reduction(std::shared_ptr<Model<Scalar>> model, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q){
    //     // Cast q and model to autodiff type
    //     VectorXreal q_real(q); // the input vector q

    //     // Compute the jacobian of the constraints
    //     VectorXreal F; // the output vector F = f(x) evaluated together with Jacobian matrix below
    //     Eigen::MatrixXd dfcdqh = jacobian(fc_function, wrt(q_real), at(q_real), F);

    // }


}  //namespace RML

#endif
