#ifndef TR_DYNAMICS_HPP
#define TR_DYNAMICS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kinematics.hpp"
#include "model.hpp"

/** \file dynamics.hpp
 * @brief Contains functions for computing the dynamics of a tinyrobotics model.
 */
namespace tinyrobotics {

    /**
     * @brief Compute the mass matrix of the tinyrobotics model.
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, nq> mass_matrix(Model<Scalar, nq>& m, const Eigen::Matrix<Scalar, nq, 1>& q) {
        m.mass_matrix.setZero();

        for (int i = 0; i < nq; i++) {
            m.Xup[i] = homogeneous_to_spatial(m.links[m.q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            m.IC[i]  = m.links[m.q_map[i]].I;
        }

        for (int i = nq - 1; i >= 0; i--) {
            if (m.parent[i] != -1) {
                m.IC[m.parent[i]] = m.IC[m.parent[i]] + m.Xup[i].transpose() * m.IC[i] * m.Xup[i];
            }
        }

        for (int i = 0; i < nq; i++) {
            m.fh                = m.IC[i] * m.links[m.q_map[i]].joint.S;
            m.mass_matrix(i, i) = m.links[m.q_map[i]].joint.S.transpose() * m.fh;
            int j               = i;
            while (m.parent[j] > -1) {
                m.fh                = m.Xup[j].transpose() * m.fh;
                j                   = m.parent[j];
                m.mass_matrix(i, j) = m.links[m.q_map[j]].joint.S.transpose() * m.fh;
                m.mass_matrix(j, i) = m.mass_matrix(i, j);
            }
        }

        return m.mass_matrix;
    }

    /**
     * @brief Compute the kinetic_energy of the tinyrobotics model.
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq The joint velocity of the robot.
     */
    template <typename Scalar, int nq>
    Scalar kinetic_energy(Model<Scalar, nq>& m,
                          const Eigen::Matrix<Scalar, nq, 1>& q,
                          const Eigen::Matrix<Scalar, nq, 1>& dq) {
        return Scalar(0.5) * dq.transpose() * mass_matrix<Scalar, nq>(m, q) * dq;
    }

    /**
     * @brief Compute the potential_energy of the tinyrobotics model.
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    Scalar potential_energy(Model<Scalar, nq>& m, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute the forward kinematics to center of mass of all links
        forward_kinematics_com(m, q);

        m.potential_energy = Scalar(0.0);
        for (auto const link : m.links) {
            m.potential_energy += -link.mass * m.gravity.transpose() * m.forward_kinematics_com[link.idx].translation();
        }
        return m.potential_energy;
    }

    /**
     * @brief Compute the total energy of the tinyrobotics model.
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq Joint velocity of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    Scalar total_energy(Model<Scalar, nq>& m,
                        const Eigen::Matrix<Scalar, nq, 1>& q,
                        const Eigen::Matrix<Scalar, nq, 1>& dq) {
        return kinetic_energy(m, q, dq) + potential_energy(m, q);
    }

    /**
     * @brief Apply external forces to the tinyrobotics model
     * @param m tinyrobotics model.
     * @param Xup The spatial transformation matrices between the ith link and its parent.
     * @param f_in The input force array of the tinyrobotics model.
     * @param f_ext The external force array to be added to the input force array.
     * @return f_out The output force array with the external forces incorporated.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Matrix<Scalar, 6, 1>> apply_external_forces(
        const Model<Scalar, nq>& m,
        const std::vector<Eigen::Matrix<Scalar, 6, 6>>& Xup,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_in,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext) {
        std::vector<Eigen::Matrix<Scalar, 6, 1>> f_out(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xa(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());
        f_out = f_in;

        for (int i = 0; i < nq; i++) {
            const auto link = m.links[m.q_map[i]];
            if (m.parent[i] == -1) {
                Xa[i] = Xup[i];
            }
            else {
                Xa[i] = Xup[i] * Xa[m.parent[i]];
            }
            f_out[i] += Xa[i].transpose().inverse() * f_ext[i];
        }
        return f_out;
    }

    /**
     * @brief Compute the forward dynamics of the tinyrobotics model via Articulated-Body Algorithm
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq Joint velocity of the robot.
     * @param tau Joint torque of the robot.
     * @param f_ext External forces acting on the robot.
     * @return Joint accelerations of the model.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> forward_dynamics(Model<Scalar, nq>& m,
                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                  const Eigen::Matrix<Scalar, nq, 1>& dq,
                                                  const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                  const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            m.vJ = m.links[m.q_map[i]].joint.S * dq(i);
            // Compute the spatial transform from the parent to the current body
            m.Xup[i] = homogeneous_to_spatial(m.links[m.q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.v[i] = m.vJ;
                m.c[i] = Eigen::Matrix<Scalar, 6, 1>::Zero();
            }
            else {
                m.v[i] = m.Xup[i] * m.v[m.parent[i]] + m.vJ;
                m.c[i] = cross_spatial(m.v[i]) * m.vJ;
            }
            m.IA[i] = m.links[m.q_map[i]].I;
            m.pA[i] = cross_motion(m.v[i]) * m.IA[i] * m.v[i];
        }

        // Apply external forces if non-zero
        if (!f_ext.empty()) {
            m.pA = apply_external_forces(m, m.Xup, m.pA, f_ext);
        }

        for (int i = nq - 1; i >= 0; i--) {
            m.U[i] = m.IA[i] * m.links[m.q_map[i]].joint.S;
            m.d[i] = m.links[m.q_map[i]].joint.S.transpose() * m.U[i];
            m.u[i] = Scalar(tau(i) - m.links[m.q_map[i]].joint.S.transpose() * m.pA[i]);
            if (m.parent[i] != -1) {
                Eigen::Matrix<Scalar, 6, 6> Ia = m.IA[i] - (m.U[i] / m.d[i]) * m.U[i].transpose();
                Eigen::Matrix<Scalar, 6, 1> pa = m.pA[i] + Ia * m.c[i] + m.U[i] * (m.u[i] / m.d[i]);
                m.IA[m.parent[i]] += m.Xup[i].transpose() * Ia * m.Xup[i];
                m.pA[m.parent[i]] += m.Xup[i].transpose() * pa;
            }
        }

        for (int i = 0; i < nq; i++) {
            if (m.parent[i] == -1) {
                m.a[i] = m.Xup[i] * -m.spatial_gravity + m.c[i];
            }
            else {
                m.a[i] = m.Xup[i] * m.a[m.parent[i]] + m.c[i];
            }
            m.ddq(i) = (m.u[i] - m.U[i].transpose() * m.a[i]) / m.d[i];
            m.a[i]   = m.a[i] + m.links[m.q_map[i]].joint.S * m.ddq(i);
        }
        return m.ddq;
    }

    /**
     * @brief Compute the forward dynamics of the tinyrobotics model via Composite-Rigid-Body Algorithm
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq Joint velocity of the robot.
     * @param tau Joint torque of the robot.
     * @param f_ext External forces acting on the robot.
     * @return Joint accelerations of the model.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> forward_dynamics_crb(Model<Scalar, nq>& m,
                                                      const Eigen::Matrix<Scalar, nq, 1>& q,
                                                      const Eigen::Matrix<Scalar, nq, 1>& dq,
                                                      const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                      const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            m.vJ = m.links[m.q_map[i]].joint.S * dq(i);
            // Compute the spatial transform from the parent to the current body
            m.Xup[i] = homogeneous_to_spatial(m.links[m.q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.v[i] = m.vJ;
                m.a[i] = m.Xup[i] * -m.spatial_gravity;
            }
            else {
                m.v[i] = m.Xup[i] * m.v[m.parent[i]] + m.vJ;
                m.a[i] = m.Xup[i] * m.a[m.parent[i]] + cross_spatial(m.v[i]) * m.vJ;
            }
            m.fvp[i] = m.links[m.q_map[i]].I * m.a[i] + cross_motion(m.v[i]) * m.links[m.q_map[i]].I * m.v[i];
            m.IC[i]  = m.links[m.q_map[i]].I;
        }

        // Apply external forces if non-zero
        if (!f_ext.empty()) {
            m.fvp = apply_external_forces(m, m.Xup, m.pA, f_ext);
        }

        for (int i = nq - 1; i >= 0; i--) {
            m.C(i, 0) = m.links[m.q_map[i]].joint.S.transpose() * m.fvp[i];
            if (m.parent[i] != -1) {
                m.fvp[m.parent[i]] = m.fvp[m.parent[i]] + m.Xup[i].transpose() * m.fvp[i];
            }
        }

        for (int i = nq - 1; i >= 0; i--) {
            if (m.parent[i] != -1) {
                m.IC[m.parent[i]] = m.IC[m.parent[i]] + m.Xup[i].transpose() * m.IC[i] * m.Xup[i];
            }
        }

        for (int i = 0; i < nq; i++) {
            m.fh                = m.IC[i] * m.links[m.q_map[i]].joint.S;
            m.mass_matrix(i, i) = m.links[m.q_map[i]].joint.S.transpose() * m.fh;
            int j               = i;
            while (m.parent[j] != -1) {
                m.fh                = m.Xup[j].transpose() * m.fh;
                j                   = m.parent[j];
                m.mass_matrix(i, j) = m.links[m.q_map[j]].joint.S.transpose() * m.fh;
                m.mass_matrix(j, i) = m.mass_matrix(i, j);
            }
        }

        return m.mass_matrix.ldlt().solve(tau - m.C);
    }

    /**
     * @brief Compute the inverse dynamics of a tinyrobotics model
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param dq Joint velocity of the robot.
     * @param f_ext External forces acting on the robot.
     * @return tau
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_dynamics(Model<Scalar, nq>& m,
                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                  const Eigen::Matrix<Scalar, nq, 1>& dq,
                                                  const Eigen::Matrix<Scalar, nq, 1>& ddq,
                                                  const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            m.vJ = m.links[m.q_map[i]].joint.S * dq(i);
            // Compute the spatial transform from the parent to the current body
            m.Xup[i] = homogeneous_to_spatial(m.links[m.q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.v[i] = m.vJ;
                m.a[i] = m.Xup[i] * -m.spatial_gravity + m.links[m.q_map[i]].joint.S * ddq(i);
            }
            else {
                m.v[i] = m.Xup[i] * m.v[m.parent[i]] + m.vJ;
                m.a[i] =
                    m.Xup[i] * m.a[m.parent[i]] + m.links[m.q_map[i]].joint.S * ddq(i) + cross_spatial(m.v[i]) * m.vJ;
            }
            m.fvp[i] = m.links[m.q_map[i]].I * m.a[i] + cross_motion(m.v[i]) * m.links[m.q_map[i]].I * m.v[i];
        }

        // Apply external forces if non-zero
        if (!f_ext.empty()) {
            m.fvp = apply_external_forces(m, m.Xup, m.pA, f_ext);
        }

        for (int i = nq - 1; i >= 0; i--) {
            m.tau(i, 0) = m.links[m.q_map[i]].joint.S.transpose() * m.fvp[i];
            if (m.parent[i] != -1) {
                m.fvp[m.parent[i]] = m.fvp[m.parent[i]] + m.Xup[i].transpose() * m.fvp[i];
            }
        }
        return m.tau;
    }

    /**
     * @brief Compute the gravity vector (generalized gravity forces) for the tinyrobotics model.
     * @param m The tinyrobotics model.
     * @param q The joint configuration of the robot.
     * @tparam Scalar Type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Gravity torque vector for the given configuration.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> gravity_torque(Model<Scalar, nq>& m, const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute inverse dynamics with zero velocities and accelerations to get only the gravitational term
        Eigen::Matrix<Scalar, nq, 1> zero = Eigen::Matrix<Scalar, nq, 1>::Zero();
        return inverse_dynamics(m, q, zero, zero);
    }

}  // namespace tinyrobotics

#endif
