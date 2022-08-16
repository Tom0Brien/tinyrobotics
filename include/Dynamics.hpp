#ifndef RML_DYNAMICS_HPP
#define RML_DYNAMICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

#include "ForwardKinematics.hpp"
#include "Model.hpp"
#include "autodiff/forward/real.hpp"
#include "autodiff/forward/real/eigen.hpp"
using namespace autodiff;

namespace RML {

    // /**
    //  * @brief Creates a vector of holonomic constraints.
    //  * @param q The joint configuration of the robot.
    //  * @return The constraint function for holonomic constraints.
    //  */
    // template <typename Scalar, int nq>
    // Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fc_function(Model<Scalar>& model,
    //                                                      const Eigen::Matrix<Scalar, nq, 1>& q,
    //                                                      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& M,
    //                                                      Scalar& V) {

    //     std::vector<Scalar> Mp;
    //     std::vector<Scalar> Jp;
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fc;

    //     // // Get the base link from the model
    //     auto base_link = model.links[model.base_link_idx];

    //     // Create over parametrised system
    //     for (auto link : model.links) {
    //         auto joint = model.joints[link.joint_idx];
    //         // Compute FK to centre of mass
    //         Eigen::Transform<Scalar, 3, Eigen::Affine> Hbm =
    //             forward_kinematics_com<Scalar, nq>(model, q, base_link.name, link.name);
    //         Eigen::Matrix<Scalar, 3, 1> rMBb = Hbm.translation();

    //         // Add links contribution to potential energy m* g* h
    //         V -= link.mass * model.gravity.transpose() * rMBb;

    //         if (link.link_idx != -1 && joint.type == RML::JointType::REVOLUTE) {
    //             // Add to mass matrix list
    //             Mp.insert(Mp.end(), {link.mass, link.mass, link.mass});
    //             // Add inertia to J matrix TODO: Need to figure out inertia contribution
    //             Jp.insert(Jp.end(), {0});
    //             // Add to constraint vector
    //             fc.conservativeResize(fc.rows() + 3);
    //             fc.tail(3) = rMBb;
    //         }
    //         else if (link.link_idx != -1 && joint.type == RML::JointType::FIXED) {
    //             // Add to mass matrix list
    //             Mp.insert(Mp.end(), {link.mass, link.mass, link.mass});
    //             // Add to constraint vector
    //             fc.conservativeResize(fc.rows() + 3);
    //             fc.tail(3) = rMBb;
    //         }
    //         else if (link.link_idx != -1 && joint.type == RML::JointType::PRISMATIC) {
    //             // Add inertia to J matrix TODO: Need to figure out inertia contribution
    //             Jp.insert(Jp.end(), {0});
    //         }
    //     }
    //     // Compute reduced system M.resize(Mp.size() + Jp.size(), Mp.size() + Jp.size());
    //     M.setZero();
    //     for (int i = 0; i < Jp.size(); i++) {
    //         M(i, i) = Jp[i];
    //     }
    //     for (int i = 0; i < Mp.size(); i++) {
    //         M(Jp.size() + i, Jp.size() + i) = Mp[i];
    //     }
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
    // template <typename Scalar, int nq>
    // void holonomic_reduction(Model<Scalar>& model,
    //                          Eigen::Matrix<Scalar, nq, 1>& q_real,
    //                          Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& M,
    //                          const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& dfcdqh) {
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mh =
    //         Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(nq, nq);
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q;
    //     Q.resize(nq + dfcdqh.rows(), dfcdqh.cols());
    //     Q.block(0, 0, nq, dfcdqh.cols()) = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq);
    //     Q.block(nq, 0, dfcdqh.rows(), dfcdqh.cols()) = dfcdqh.template cast<Scalar>();
    //     Mh                                           = Q.transpose() * M * Q;
    //     M.resize(nq, nq);
    //     M = Mh;
    // }

    // /**
    //  * @brief Compute the mass matrix, coriolis and gravity matrices of the robot model.
    //  * @param model The robot model.
    //  * @param q The joint configuration of the robot.
    //  */
    // template <typename Scalar, int nq>
    // void compute_dynamics(Model<Scalar>& model,
    //                       const Eigen::Matrix<Scalar, nq, 1>& q,
    //                       Eigen::Matrix<Scalar, nq, nq>& Mh,
    //                       Eigen::Matrix<Scalar, nq, nq>& Ch,
    //                       Eigen::Matrix<Scalar, nq, 1>& g,
    //                       Scalar& Vh) {
    //     // Create the mass matrix, inertia matrix, constraint vector and potential energy matrices
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic> M;
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc;
    //     autodiff::real V = 0;

    //     // Cast to autodiff::real type
    //     Eigen::Matrix<autodiff::real, nq, 1> q_real(q);
    //     RML::Model<autodiff::real> autodiff_model = model.template cast<autodiff::real>();

    //     // Compute the holonomic constraint vector and its jacobian
    //     Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> F;
    //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> dfcdqh =
    //         jacobian(fc_function<autodiff::real, nq>, wrt(q_real), at(autodiff_model, q_real, M, V), F);

    //     // // Compute the mass matrix via holonomic constraint elimination
    //     // holonomic_reduction<autodiff::real, nq>(autodiff_model, q_real, M, dfcdqh);

    //     // TODO:  Compute the coriolis
    //     // Ch.setZero();
    //     // TODO: Compute the gravity torque vector
    //     // g.setZero();
    //     // Cast back to Scalar
    //     // Mh = M.template cast<Scalar>();
    //     // Vh = val(V);
    // };

    /**
     * @brief Compute the mass matrix of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void mass_matrix(Model<Scalar>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Update the model with the current joint configuration
        model.results.q = q;
        // Reset the mass matrix and potential energy
        model.results.M.setZero();
        model.results.V = 0;

        Eigen::Matrix<Scalar, 6, 6> Mi;
        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];
        for (int i = 0; i < model.n_links; i++) {
            Mi.setZero();
            // Compute the rotation between the ith link and the base
            Eigen::Matrix<Scalar, 3, 3> R0i = rotation(model, q, base_link.name, model.links[i].name);
            // Insert the mass of the link into the top 3 diagonals
            Mi.block(0, 0, 3, 3) = model.links[i].mass * Eigen::Matrix<Scalar, 3, 3>::Identity();
            // Insert the inertia of the link into the bottom 3 diagonals
            Mi.block(3, 3, 3, 3) = R0i * model.links[i].inertia * R0i.transpose();
            // Compute the geometric jacobian of the links center of mass with respect to the base
            Eigen::Matrix<Scalar, 6, nq> Jci = geometric_jacobian_com(model, q, model.links[i].name);
            // Compute the contribution to the mass matrix of the link
            model.results.M += Jci.transpose() * Mi * Jci;
            // Compute the contribution to the potential energy of the link
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hbi_c =
                forward_kinematics_com<Scalar, nq>(model, q, model.base_link_idx, model.links[i].link_idx);
            Eigen::Matrix<Scalar, 3, 1> rMIi_c = Hbi_c.translation();
            model.results.V += -model.links[i].mass * model.gravity.transpose() * rMIi_c;
        }
    }
    /**
     *
     * @brief Compute the coriolis matrix model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void coriolis_matrix(Model<Scalar>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute the mass matrix
        mass_matrix<Scalar, nq>(model, q);
        // TODO: Compute the coriolis matrix
    }

    /**
     * @brief Compute the gravity matrix of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void gravity_torque(Model<Scalar>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute the mass matrix
        mass_matrix<Scalar, nq>(model, q);
        // TODO: Compute the gravity matrix
    }

    /**
     * @brief Compute the kinetic_energy of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void kinetic_energy(Model<Scalar>& model,
                        const Eigen::Matrix<Scalar, nq, 1>& q,
                        const Eigen::Matrix<Scalar, nq, 1>& dq) {
        // Update the model with the current joint configuration and velocity
        model.results.q  = q;
        model.results.dq = dq;
        // Compute the mass matrix
        mass_matrix<Scalar, nq>(model, q);
        // Compute the kinetic energy
        model.results.T = 0.5 * model.results.dq.transpose() * model.results.M * model.results.dq;
    }

    /**
     * @brief Compute the potential_energy of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void potential_energy(Model<Scalar>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Update the model with the current joint configuration
        model.results.q = q;
        // Reset the potential energy
        model.results.V = 0;
        // Compute the potential energy
        for (int i = 0; i < model.n_links; i++) {
            // Compute the contribution to the potential energy of the link
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hbi_c =
                forward_kinematics_com<Scalar, nq>(model, q, model.base_link_idx, model.links[i].link_idx);
            Eigen::Matrix<Scalar, 3, 1> rMIi_c = Hbi_c.translation();
            model.results.V += -model.links[i].mass * model.gravity.transpose() * rMIi_c;
        }
    }

    /**
     * @brief Compute the hamiltonian of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param dq The joint velocity of the robot.
     */
    template <typename Scalar, int nq>
    void hamiltonian(Model<Scalar>& model,
                     const Eigen::Matrix<Scalar, nq, 1>& q,
                     const Eigen::Matrix<Scalar, nq, 1>& dq) {
        // Update the model with the current joint configuration and velocity
        model.results.q  = q;
        model.results.dq = dq;
        // Compute the kinetic energy
        kinetic_energy<Scalar, nq>(model, q, dq);
        // Compute the potential energy
        potential_energy<Scalar, nq>(model, q);
        // Compute the hamiltonian
        model.results.H = model.results.T + model.results.V;
    }

}  // namespace RML

#endif
