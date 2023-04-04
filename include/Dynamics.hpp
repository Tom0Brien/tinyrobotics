#ifndef TR_DYNAMICS_HPP
#define TR_DYNAMICS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include "Kinematics.hpp"
#include "Model.hpp"

/** \file Dynamics.hpp
 * @brief Contains functions for computing the dynamics of a tinyrobotics model.
 */
namespace tinyrobotics {
    /**
     * @brief Compute the mass matrix of the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, nq> mass_matrix(Model<Scalar, nq>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Reset the mass matrix and potential energy
        model.data.M.setZero();
        model.data.V = 0;
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
            model.data.M += Jci.transpose() * Mi * Jci;
            // Compute the contribution to the potential energy of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbi_c =
                forward_kinematics_com<Scalar, nq>(model, q, model.base_link_idx, model.links[i].idx);
            Eigen::Matrix<Scalar, 3, 1> rMIi_c = Hbi_c.translation();
            model.data.V += -model.links[i].mass * model.gravity.transpose() * rMIi_c;
        }
        return model.data.M;
    }

    /**
     * @brief Compute the kinetic_energy of the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq The joint velocity of the robot.
     */
    template <typename Scalar, int nq>
    void kinetic_energy(Model<Scalar, nq>& model,
                        const Eigen::Matrix<Scalar, nq, 1>& q,
                        const Eigen::Matrix<Scalar, nq, 1>& dq) {

        // Compute the mass matrix
        mass_matrix<Scalar, nq>(model, q);
        // Compute the kinetic energy
        model.data.T = 0.5 * dq.transpose() * model.data.M * dq;
    }

    /**
     * @brief Compute the potential_energy of the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    void potential_energy(Model<Scalar, nq>& model, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Reset the potential energy
        model.data.V = 0;
        // Compute the potential energy
        for (int i = 0; i < model.n_links; i++) {
            // Compute the contribution to the potential energy of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbi_c =
                forward_kinematics_com<Scalar, nq>(model, q, model.base_link_idx, model.links[i].idx);
            Eigen::Matrix<Scalar, 3, 1> rMIi_c = Hbi_c.translation();
            model.data.V += -model.links[i].mass * model.gravity.transpose() * rMIi_c;
        }
    }

    /**
     * @brief Compute the total energy of the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param p Joint momentum of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 1, 1> total_energy(Model<Scalar, nq>& model,
                                             const Eigen::Matrix<Scalar, nq, 1>& q,
                                             const Eigen::Matrix<Scalar, nq, 1>& p) {
        // Compute the mass matrix and potential energy
        mass_matrix<Scalar, nq>(model, q);

        // Compute the inverse of the mass matrix
        Eigen::Matrix<Scalar, nq, nq> b    = Eigen::Matrix<Scalar, nq, nq>::Identity();
        Eigen::Matrix<Scalar, nq, nq> Minv = model.data.M.ldlt().solve(b);
        model.data.Minv                    = Minv;

        // Compute the kinetic energy
        model.data.T = Scalar(0.5 * p.transpose() * Minv * p);

        // Compute the total energy
        Eigen::Matrix<Scalar, 1, 1> H = Eigen::Matrix<Scalar, 1, 1>(model.data.T + model.data.V);
        return H;
    }

    /**
     * @brief Apply external forces to the tinyrobotics model
     * @param model tinyrobotics model.
     * @param f_in The input force array of the tinyrobotics model.
     * @param f_ext The external force array to be added to the input force array.
     * @return f_out The output force array with the external forces incorporated.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Matrix<Scalar, 6, 1>> apply_external_forces(
        const Model<Scalar, nq>& model,
        const std::vector<Eigen::Matrix<Scalar, 6, 6>>& Xup,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_in,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext) {
        std::vector<Eigen::Matrix<Scalar, 6, 1>> f_out(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xa(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        f_out = f_in;
        for (int i = 0; i < nq; i++) {
            const auto link = model.links[model.q_idx[i]];
            if (model.parent[i] == -1) {
                Xa[i] = Xup[i];
            }
            else {
                Xa[i] = Xup[i] * Xa[model.parent[i]];
            }
            f_out[i] += Xa[i].transpose().inverse() * f_ext[i];
        }
        return f_out;
    }


    /**
     * @brief Compute the forward dynamics of the tinyrobotics model via Articulated-Body Algorithm
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param qd Joint velocity of the robot.
     * @param tau Joint torque of the robot.
     * @param f_ext External forces acting on the robot.
     * @return Joint accelerations of the model.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> forward_dynamics(Model<Scalar, nq>& m,
                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                  const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                  const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                  const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        // Initialize the vectors for the algorithm
        m.data.gravity.tail(3) = m.gravity;
        // First pass
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            auto link                      = m.links[m.q_idx[i]];
            m.data.S[i]                    = link.joint.S;
            Eigen::Matrix<Scalar, 6, 1> vJ = m.data.S[i] * qd(i);
            // Get transform from body to parent
            auto T = link.joint.get_parent_to_child_transform(q(i));
            // Compute the spatial transform from the parent to the current body
            m.data.Xup[i] = homogeneous_to_spatial(T.inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.data.v[i] = vJ;
                m.data.c[i] = Eigen::Matrix<Scalar, 6, 1>::Zero();
            }
            else {
                m.data.v[i] = m.data.Xup[i] * m.data.v[m.parent[i]] + vJ;
                m.data.c[i] = cross_spatial(m.data.v[i]) * vJ;
            }
            m.data.IA[i] = link.I;
            m.data.pA[i] = cross_motion(m.data.v[i]) * m.data.IA[i] * m.data.v[i];
        }

        // Apply external forces if non-zero
        if (f_ext.size() != 0) {
            m.data.pA = apply_external_forces(m, m.data.Xup, m.data.pA, f_ext);
        }

        // Second pass
        for (int i = nq - 1; i >= 0; i--) {
            m.data.U[i] = m.data.IA[i] * m.data.S[i];
            m.data.d[i] = m.data.S[i].transpose() * m.data.U[i];
            m.data.u[i] = Scalar(tau(i) - m.data.S[i].transpose() * m.data.pA[i]);
            if (m.parent[i] != -1) {
                Eigen::Matrix<Scalar, 6, 6> Ia = m.data.IA[i] - (m.data.U[i] / m.data.d[i]) * m.data.U[i].transpose();
                Eigen::Matrix<Scalar, 6, 1> pa =
                    m.data.pA[i] + Ia * m.data.c[i] + m.data.U[i] * (m.data.u[i] / m.data.d[i]);
                m.data.IA[m.parent[i]] += m.data.Xup[i].transpose() * Ia * m.data.Xup[i];
                m.data.pA[m.parent[i]] += m.data.Xup[i].transpose() * pa;
            }
        }

        // Third pass
        for (int i = 0; i < nq; i++) {
            if (m.parent[i] == -1) {
                m.data.a[i] = m.data.Xup[i] * -m.data.gravity + m.data.c[i];
            }
            else {
                m.data.a[i] = m.data.Xup[i] * m.data.a[m.parent[i]] + m.data.c[i];
            }
            m.data.ddq(i) = (m.data.u[i] - m.data.U[i].transpose() * m.data.a[i]) / m.data.d[i];
            m.data.a[i]   = m.data.a[i] + m.data.S[i] * m.data.ddq(i);
        }
        return m.data.ddq;
    }

}  // namespace tinyrobotics

#endif
