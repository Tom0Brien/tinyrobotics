#ifndef TR_DATA_HPP
#define TR_DATA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

/** \file Data.hpp
 * @brief Contains the Data struct which stores the results of various model algorithms.
 */
namespace tr::data {

    /**
     * @brief A data struct for storing results of various model algorithms.
     * @tparam Scalar Scalar type of the associated model.
     * @tparam nq Number of configuration coordinates (number of degrees of freedom)
     */
    template <typename Scalar, int nq>
    struct Data {

        /// @brief Joint configuration.
        Eigen::Matrix<Scalar, nq, 1> q;

        /// @brief Joint velocity.
        Eigen::Matrix<Scalar, nq, 1> dq;

        /// @brief Joint acceleration.
        Eigen::Matrix<Scalar, nq, 1> ddq;

        /// @brief Joint Momentum.
        Eigen::Matrix<Scalar, nq, 1> p;

        /// @brief Joint torque.
        Eigen::Matrix<Scalar, nq, 1> tau;

        /// @brief Mass matrix.
        Eigen::Matrix<Scalar, nq, nq> M = Eigen::Matrix<Scalar, nq, nq>::Zero();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mr;

        /// @brief Inverse mass matrix.
        Eigen::Matrix<Scalar, nq, nq> Minv = Eigen::Matrix<Scalar, nq, nq>::Zero();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mrinv;

        /// @brief Coriolis matrix.
        Eigen::Matrix<Scalar, nq, nq> C = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Gravity torque vector.
        Eigen::Matrix<Scalar, nq, 1> g;

        /// @brief Input mapping matrix.
        Eigen::Matrix<Scalar, nq, nq> Gp = Eigen::Matrix<Scalar, nq, nq>::Identity();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Gr;

        /// @brief Damping matrix.
        Eigen::Matrix<Scalar, nq, nq> Dp = Eigen::Matrix<Scalar, nq, nq>::Zero();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Dr;

        /// @brief Constraint jacobian
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jc;

        /// @brief Constraint jacobian left annihilator
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jcp;

        /// @brief The kinetic co-energy.
        Scalar T = 0;

        /// @brief The potential energy.
        Scalar V = 0;

        /// @brief The hamiltonian (total energy).
        Scalar H = 0;

        /// @brief The dynamics dx_dt
        Eigen::Matrix<Scalar, nq + nq, 1> dx_dt;

        /// @brief Vector of forward kinematics Data.
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> FK = {};

        /// @brief Number of reduced momentum states
        int nr = 0;

        /// @brief Vector of redundant momentum states
        int nz = 0;

        /**
         * @brief Casts the data to a new scalar type.
         * @tparam NewScalar scalar type to cast the data to.
         * @return Data with new scalar type.
         */
        template <typename NewScalar>
        Data<NewScalar, nq> cast() {
            Data<NewScalar, nq> new_res;
            new_res.q     = q.template cast<NewScalar>();
            new_res.dq    = dq.template cast<NewScalar>();
            new_res.ddq   = ddq.template cast<NewScalar>();
            new_res.p     = p.template cast<NewScalar>();
            new_res.tau   = tau.template cast<NewScalar>();
            new_res.M     = M.template cast<NewScalar>();
            new_res.Minv  = Minv.template cast<NewScalar>();
            new_res.C     = C.template cast<NewScalar>();
            new_res.g     = g.template cast<NewScalar>();
            new_res.Gp    = Gp.template cast<NewScalar>();
            new_res.Dp    = Dp.template cast<NewScalar>();
            new_res.T     = NewScalar(T);
            new_res.V     = NewScalar(V);
            new_res.H     = NewScalar(H);
            new_res.dx_dt = dx_dt.template cast<NewScalar>();
            for (int i = 0; i < FK.size(); i++) {
                new_res.FK[i] = FK[i].template cast<NewScalar>();
            }
            return new_res;
        }
    };
}  // namespace tr::data

#endif