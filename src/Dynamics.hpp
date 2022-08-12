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

    /**
     * @brief Creates a vector of holonomic constraints.
     * @param q The joint configuration of the robot.
     * @return The constraint function for holonomic constraints.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc_function(
        Model<Scalar, nq>& model,
        const Eigen::Matrix<Scalar, nq, 1>& q,
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic>& M,
        autodiff::real& V) {

        std::vector<Scalar> Mp;
        std::vector<Scalar> Jp;
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fc;

        // Create over parametrised system
        for (auto link : model.links) {
            // Compute FK to centre of mass
            Eigen::Transform<autodiff::real, 3, Eigen::Affine> Hbm =
                forward_kinematics_com<autodiff::real, nq>(model, q, model.base_link->name, link->name);
            Eigen::Matrix<autodiff::real, 3, 1> rMBb = Hbm.translation();

            // Add links contribution to potential energy m*g*h
            V -= link->mass * model.gravity.transpose() * rMBb;

            if (link->joint != nullptr && link->joint->type == RML::JointType::REVOLUTE) {
                // Add to mass matrix list
                Mp.insert(Mp.end(), {link->mass, link->mass, link->mass});
                // Add inertia to J matrix TODO: Need to figure out inertia contribution
                Jp.insert(Jp.end(), {0});
                // Add to constraint vector
                fc.conservativeResize(fc.rows() + 3);
                fc.tail(3) = rMBb;
            }
            else if (link->joint != nullptr && link->joint->type == RML::JointType::FIXED) {
                // Add to mass matrix list
                Mp.insert(Mp.end(), {link->mass, link->mass, link->mass});
                // Add to constraint vector
                fc.conservativeResize(fc.rows() + 3);
                fc.tail(3) = rMBb;
            }
            else if (link->joint != nullptr && link->joint->type == RML::JointType::PRISMATIC) {
                // Add inertia to J matrix TODO: Need to figure out inertia contribution
                Jp.insert(Jp.end(), {0});
            }
        }
        // Compute reduced system
        M.resize(Mp.size() + Jp.size(), Mp.size() + Jp.size());
        M.setZero();
        for (int i = 0; i < Jp.size(); i++) {
            M(i, i) = Jp[i];
        }
        for (int i = 0; i < Mp.size(); i++) {
            M(Jp.size() + i, Jp.size() + i) = Mp[i];
        }
        return fc;
    }

    /**
     * @brief Eliminates holonomic constraints from the dynamic equations of motion via the appropriate selection
     * of generalised coordinates.
     * @param model The robot model.
     * @param q_real The joint configuration of the robot.
     * @param fc The holonomic constraints.
     * @return The mass matrix of the robot model.
     */
    template <typename Scalar, int nq>
    void holonomic_reduction(Model<Scalar, nq>& model,
                             Eigen::Matrix<Scalar, nq, 1>& q_real,
                             Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& M,
                             const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& dfcdqh) {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mh =
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(nq, nq);
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q;
        Q.resize(nq + dfcdqh.rows(), dfcdqh.cols());
        Q.block(0, 0, nq, dfcdqh.cols()) = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq);
        Q.block(nq, 0, dfcdqh.rows(), dfcdqh.cols()) = dfcdqh.template cast<Scalar>();
        Mh                                           = Q.transpose() * M * Q;
        M.resize(nq, nq);
        M = Mh;
    }

    /**
     * @brief Compute the mass matrix, coriolis and gravity matrices of the robot model.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void compute_dynamics(Model<Scalar, nq>& model,
                          const Eigen::Matrix<Scalar, nq, 1>& q,
                          Eigen::Matrix<Scalar, nq, nq>& Mh,
                          Eigen::Matrix<Scalar, nq, nq>& Ch,
                          Eigen::Matrix<Scalar, nq, 1>& g,
                          Scalar& Vh) {
        // Create the mass matrix, inertia matrix, constraint vector and potential energy matrices
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic> M;
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc;
        autodiff::real V = 0;

        // Cast to autodiff::real type
        Eigen::Matrix<autodiff::real, nq, 1> q_real(q);
        RML::Model<autodiff::real, nq> autodiff_model = model.template cast<autodiff::real, nq>();

        // Compute the holonomic constraint vector and its jacobian
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> F;
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> dfcdqh =
            jacobian(fc_function<autodiff::real, nq>, wrt(q_real), at(autodiff_model, q_real, M, V), F);

        // Compute the mass matrix via holonomic constraint elimination
        holonomic_reduction<autodiff::real, nq>(autodiff_model, q_real, M, dfcdqh);

        // TODO:  Compute the coriolis
        Ch.setZero();
        // TODO: Compute the gravity torque vector
        g.setZero();
        // Cast back to Scalar
        Mh = M.template cast<Scalar>();
        Vh = val(V);
    };
}  // namespace RML

#endif
