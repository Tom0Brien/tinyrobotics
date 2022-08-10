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
     * @brief Constraint function for holonomic constraints.
     * @param q The joint configuration of the robot.
     * @return The constraint function for holonomic constraints.
     */
    Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic> fc_function(
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1>& fc) {
        auto fc_ = fc;
        return fc_;
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
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> holonomic_reduction(
        Model<Scalar, nq>& model,
        Eigen::Matrix<autodiff::real, nq, 1>& q_real,
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic>& M,
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1>& fc) {
        // Compute the jacobian of the constraints
        VectorXreal F;  // the output vector F = f(x) evaluated together with Jacobian matrix below

        std::cout << "fc = " << std::endl << fc << std::endl;
        Eigen::MatrixXd dfcdqh = jacobian(fc_function, wrt(q_real), at(fc), F);
        // std::cout << "dfcdqh = " << std::endl << dfcdqh << std::endl;
        // std::cout << "F = " << std::endl << F << std::endl;

        int n = q_real.rows();

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mh =
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        // Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Reduction;
        // Reduction.resize(nq + dfcdqh.rows(), dfcdqh.cols());
        // Reduction.block(0, 0, nq, dfcdqh.cols()) =
        //     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq);
        // Reduction.block(nq, 0, dfcdqh.rows(), dfcdqh.cols()) = dfcdqh.template cast<Scalar>();
        // std::cout << "Reduction = " << std::endl << Reduction << std::endl;

        // Mh = Reduction.transpose() * M * Reduction;
        // std::cout << "Mh = " << std::endl << Mh << std::endl;
        return Mh;
    }

    /**
     * @brief Construct a new Model object from URDF file description.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    void compute_dynamics(Model<Scalar, nq>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {

        std::vector<Scalar> Mp;
        std::vector<Scalar> Jp;
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> fc;
        Scalar V = 0;

        // Cast to autodiff::real
        Eigen::Matrix<autodiff::real, nq, 1> q_real(q);
        Model<autodiff::real, nq> model_real = model.template cast<autodiff::real, nq>();

        // Create over parametrised system
        for (auto link : model.links) {
            // Compute FK to centre of mass
            Eigen::Transform<autodiff::real, 3, Eigen::Affine> Hbm =
                forward_kinematics_com<autodiff::real, nq>(model_real, q_real, model_real.base_link->name, link->name);
            Eigen::Matrix<autodiff::real, 3, 1> rMBb = Hbm.translation();

            // // Add links contribution to potential energy m*g*h
            V = V - link->mass * model.gravity.transpose() * rMBb.cast<Scalar>();

            if (link->joint != nullptr && link->joint->type == RML::JointType::REVOLUTE) {
                // Add to mass matrix list
                Mp.insert(Mp.end(), {link->mass, link->mass, link->mass});
                // Add inertia to J matrix TODO: Probably need to actually load inertia information
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
                // Add inertia to J matrix TODO: Probably need to actually load inertia information
                Jp.insert(Jp.end(), {0});
            }
        }
        // Compute reduced system
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, Eigen::Dynamic> M;
        M.resize(Mp.size() + Jp.size(), Mp.size() + Jp.size());
        M.setZero();
        for (int i = 0; i < Jp.size(); i++) {
            M(i, i) = Jp[i];
        }
        for (int i = 0; i < Mp.size(); i++) {
            M(Jp.size() + i, Jp.size() + i) = Mp[i];
        }

        auto Mh = holonomic_reduction(model, q_real, M, fc);
    };


}  // namespace RML

#endif
