#ifndef RML_DATA_HPP
#define RML_DATA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RML {

    /**
     * @brief A data struct for storing results of various model algorithms.
     */
    template <typename Scalar, int nq>
    struct Data {

        /// @brief Joint configuration.
        Eigen::Matrix<Scalar, nq, 1> q;

        /// @brief Joint velocity.
        Eigen::Matrix<Scalar, nq, 1> dq;

        /// @brief Joint acceleration.
        Eigen::Matrix<Scalar, nq, 1> ddq;

        /// @brief Joint Momentum
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
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Affine>> FK = {};

        /// @brief Number of reduced momentum states
        int nr = 0;

        /// @brief Vector of redundant momentum states
        int nz = 0;

        /**
         * @brief Resize all matrices to the given size.
         * @param n The size to resize all matrices to.
         */
        void resize(int n) {
            // q.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // dq.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // ddq.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // p.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // tau.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // M.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            // Minv.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            // C.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            // g.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            // Gp.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(n, n));
            // Dp.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            // dx_dt.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n + n, 1));
        }

        /**
         * @brief Cast to NewScalar type.
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
}  // namespace RML

#endif