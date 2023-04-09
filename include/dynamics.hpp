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
        // First pass
        for (int i = 0; i < nq; i++) {
            // Get transform from body to parent
            Eigen::Transform<Scalar, 3, Eigen::Isometry> T =
                m.links[m.q_idx[i]].joint.get_parent_to_child_transform(q(i));
            // Compute the spatial transform from the parent to the current body
            m.data.Xup[i] = homogeneous_to_spatial(T.inverse());
            m.data.IC[i]  = m.links[m.q_idx[i]].I;
        }

        // Second pass
        for (int i = nq - 1; i >= 0; i--) {
            if (m.parent[i] != -1) {
                m.data.IC[m.parent[i]] =
                    m.data.IC[m.parent[i]] + m.data.Xup[i].transpose() * m.data.IC[i] * m.data.Xup[i];
            }
        }

        // Third pass
        m.data.mass_matrix.setZero();
        for (int i = 0; i < nq; i++) {
            m.data.fh                = m.data.IC[i] * m.links[m.q_idx[i]].joint.S;
            m.data.mass_matrix(i, i) = m.links[m.q_idx[i]].joint.S.transpose() * m.data.fh;
            int j                    = i;
            while (m.parent[j] > -1) {
                m.data.fh                = m.data.Xup[j].transpose() * m.data.fh;
                j                        = m.parent[j];
                m.data.mass_matrix(i, j) = m.links[m.q_idx[j]].joint.S.transpose() * m.data.fh;
                m.data.mass_matrix(j, i) = m.data.mass_matrix(i, j);
            }
        }

        return m.data.mass_matrix;
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

        // Compute the mass matrix
        mass_matrix<Scalar, nq>(m, q);
        // Compute the kinetic energy
        m.data.kinetic_energy = 0.5 * dq.transpose() * m.data.mass_matrix * dq;
        return m.data.kinetic_energy;
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
        // Reset the potential energy
        m.data.potential_energy = 0;
        // Compute the potential energy
        for (int i = 0; i < m.n_links; i++) {
            // Compute the contribution to the potential energy of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbi_c =
                forward_kinematics_com<Scalar, nq>(m, q, m.base_link_idx, m.links[i].idx);
            Eigen::Matrix<Scalar, 3, 1> rMIi_c = Hbi_c.translation();
            m.data.potential_energy += -m.links[i].mass * m.gravity.transpose() * rMIi_c;
        }
        return m.data.potential_energy;
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
        // Compute the kinetic energy
        kinetic_energy(m, q, dq);

        // Compute the potential energy
        potential_energy(m, q);

        // Compute and return the total energy (kinetic + potential)
        return m.data.kinetic_energy + m.data.potential_energy;
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
            const auto link = m.links[m.q_idx[i]];
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

        // First pass: compute the spatial acceleration of each body
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            m.links[m.q_idx[i]].joint.S    = m.links[m.q_idx[i]].joint.S;
            Eigen::Matrix<Scalar, 6, 1> vJ = m.links[m.q_idx[i]].joint.S * qd(i);
            // Get transform from body to parent
            m.data.T = m.links[m.q_idx[i]].joint.get_parent_to_child_transform(q(i));
            // Compute the spatial transform from the parent to the current body
            m.data.Xup[i] = homogeneous_to_spatial(m.data.T.inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.data.v[i] = vJ;
                m.data.c[i] = Eigen::Matrix<Scalar, 6, 1>::Zero();
            }
            else {
                m.data.v[i] = m.data.Xup[i] * m.data.v[m.parent[i]] + vJ;
                m.data.c[i] = cross_spatial(m.data.v[i]) * vJ;
            }
            m.data.IA[i] = m.links[m.q_idx[i]].I;
            m.data.pA[i] = cross_motion(m.data.v[i]) * m.data.IA[i] * m.data.v[i];
        }

        // Apply external forces if non-zero
        if (f_ext.size() != 0) {
            m.data.pA = apply_external_forces(m, m.data.Xup, m.data.pA, f_ext);
        }

        // Second pass: compute the spatial force acting on each body
        for (int i = nq - 1; i >= 0; i--) {
            m.data.U[i] = m.data.IA[i] * m.links[m.q_idx[i]].joint.S;
            m.data.d[i] = m.links[m.q_idx[i]].joint.S.transpose() * m.data.U[i];
            m.data.u[i] = Scalar(tau(i) - m.links[m.q_idx[i]].joint.S.transpose() * m.data.pA[i]);
            if (m.parent[i] != -1) {
                Eigen::Matrix<Scalar, 6, 6> Ia = m.data.IA[i] - (m.data.U[i] / m.data.d[i]) * m.data.U[i].transpose();
                Eigen::Matrix<Scalar, 6, 1> pa =
                    m.data.pA[i] + Ia * m.data.c[i] + m.data.U[i] * (m.data.u[i] / m.data.d[i]);
                m.data.IA[m.parent[i]] += m.data.Xup[i].transpose() * Ia * m.data.Xup[i];
                m.data.pA[m.parent[i]] += m.data.Xup[i].transpose() * pa;
            }
        }

        // Third pass: compute the joint accelerations
        for (int i = 0; i < nq; i++) {
            if (m.parent[i] == -1) {
                m.data.a[i] = m.data.Xup[i] * -m.data.spatial_gravity + m.data.c[i];
            }
            else {
                m.data.a[i] = m.data.Xup[i] * m.data.a[m.parent[i]] + m.data.c[i];
            }
            m.data.ddq(i) = (m.data.u[i] - m.data.U[i].transpose() * m.data.a[i]) / m.data.d[i];
            m.data.a[i]   = m.data.a[i] + m.links[m.q_idx[i]].joint.S * m.data.ddq(i);
        }
        return m.data.ddq;
    }

    /**
     * @brief Compute the forward dynamics of the tinyrobotics model via Composite-Rigid-Body Algorithm
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param qd Joint velocity of the robot.
     * @param tau Joint torque of the robot.
     * @param f_ext External forces acting on the robot.
     * @return Joint accelerations of the model.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> forward_dynamics_crb(Model<Scalar, nq>& m,
                                                      const Eigen::Matrix<Scalar, nq, 1>& q,
                                                      const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                      const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                      const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        // First pass
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            Eigen::Matrix<Scalar, 6, 1> vJ = m.links[m.q_idx[i]].joint.S * qd(i);
            // Get transform from body to parent
            Eigen::Transform<Scalar, 3, Eigen::Isometry> T =
                m.links[m.q_idx[i]].joint.get_parent_to_child_transform(q(i));
            // Compute the spatial transform from the parent to the current body
            m.data.Xup[i] = homogeneous_to_spatial(T.inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.data.v[i]   = vJ;
                m.data.avp[i] = m.data.Xup[i] * -m.data.spatial_gravity;
            }
            else {
                m.data.v[i]   = m.data.Xup[i] * m.data.v[m.parent[i]] + vJ;
                m.data.avp[i] = m.data.Xup[i] * m.data.avp[m.parent[i]] + cross_spatial(m.data.v[i]) * vJ;
            }
            m.data.fvp[i] =
                m.links[m.q_idx[i]].I * m.data.avp[i] + cross_motion(m.data.v[i]) * m.links[m.q_idx[i]].I * m.data.v[i];
            m.data.IC[i] = m.links[m.q_idx[i]].I;
        }

        // Apply external forces if non-zero
        if (f_ext.size() != 0) {
            m.data.fvp = apply_external_forces(m, m.data.Xup, m.data.pA, f_ext);
        }

        // Second pass
        for (int i = nq - 1; i >= 0; i--) {
            m.data.C(i, 0) = m.links[m.q_idx[i]].joint.S.transpose() * m.data.fvp[i];
            if (m.parent[i] != -1) {
                m.data.fvp[m.parent[i]] = m.data.fvp[m.parent[i]] + m.data.Xup[i].transpose() * m.data.fvp[i];
            }
        }

        // Third pass
        for (int i = nq - 1; i >= 0; i--) {
            if (m.parent[i] != -1) {
                m.data.IC[m.parent[i]] =
                    m.data.IC[m.parent[i]] + m.data.Xup[i].transpose() * m.data.IC[i] * m.data.Xup[i];
            }
        }

        for (int i = 0; i < nq; i++) {
            m.data.fh                = m.data.IC[i] * m.links[m.q_idx[i]].joint.S;
            m.data.mass_matrix(i, i) = m.links[m.q_idx[i]].joint.S.transpose() * m.data.fh;
            int j                    = i;
            while (m.parent[j] != -1) {
                m.data.fh                = m.data.Xup[j].transpose() * m.data.fh;
                j                        = m.parent[j];
                m.data.mass_matrix(i, j) = m.links[m.q_idx[j]].joint.S.transpose() * m.data.fh;
                m.data.mass_matrix(j, i) = m.data.mass_matrix(i, j);
            }
        }

        // Compute qdd with inverse mass using LDLT
        Eigen::Matrix<Scalar, nq, 1> qdd = m.data.mass_matrix.ldlt().solve(tau - m.data.C);

        return qdd;
    }

    /**
     * @brief Compute the inverse dynamics of a tinyrobotics model
     * @param m tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param qd Joint velocity of the robot.
     * @param f_ext External forces acting on the robot.
     * @return tau
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_dynamics(Model<Scalar, nq>& m,
                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                  const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                  const Eigen::Matrix<Scalar, nq, 1>& qdd,
                                                  const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {}) {
        // First pass
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            m.links[m.q_idx[i]].joint.S    = m.links[m.q_idx[i]].joint.S;
            Eigen::Matrix<Scalar, 6, 1> vJ = m.links[m.q_idx[i]].joint.S * qd(i);
            // Get transform from body to parent
            Eigen::Transform<Scalar, 3, Eigen::Isometry> T =
                m.links[m.q_idx[i]].joint.get_parent_to_child_transform(q(i));
            // Compute the spatial transform from the parent to the current body
            m.data.Xup[i] = homogeneous_to_spatial(T.inverse());
            // Check if the m.parent link is the base link
            if (m.parent[i] == -1) {
                m.data.v[i] = vJ;
                m.data.a[i] = m.data.Xup[i] * -m.data.spatial_gravity + m.links[m.q_idx[i]].joint.S * qdd(i);
            }
            else {
                m.data.v[i] = m.data.Xup[i] * m.data.v[m.parent[i]] + vJ;
                m.data.a[i] = m.data.Xup[i] * m.data.a[m.parent[i]] + m.links[m.q_idx[i]].joint.S * qdd(i)
                              + cross_spatial(m.data.v[i]) * vJ;
            }
            m.data.fvp[i] =
                m.links[m.q_idx[i]].I * m.data.a[i] + cross_motion(m.data.v[i]) * m.links[m.q_idx[i]].I * m.data.v[i];
        }

        // Apply external forces if non-zero
        if (f_ext.size() != 0) {
            m.data.fvp = apply_external_forces(m, m.data.Xup, m.data.pA, f_ext);
        }

        // Second pass
        for (int i = nq - 1; i >= 0; i--) {
            m.data.tau(i, 0) = m.links[m.q_idx[i]].joint.S.transpose() * m.data.fvp[i];
            if (m.parent[i] != -1) {
                m.data.fvp[m.parent[i]] = m.data.fvp[m.parent[i]] + m.data.Xup[i].transpose() * m.data.fvp[i];
            }
        }
        return m.data.tau;
    }

}  // namespace tinyrobotics

#endif
