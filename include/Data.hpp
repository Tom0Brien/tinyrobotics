#ifndef TR_DATA_HPP
#define TR_DATA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

/** \file Data.hpp
 * @brief Contains the Data struct which stores the results of various model algorithms.
 */
namespace tinyrobotics {

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

        /// @brief Joint torque/force.
        Eigen::Matrix<Scalar, nq, 1> tau;

        /// @brief Mass matrix.
        Eigen::Matrix<Scalar, nq, nq> mass_matrix = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Inverse mass matrix.
        Eigen::Matrix<Scalar, nq, nq> inv_mass_matrix = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Coriolis matrix.
        Eigen::Matrix<Scalar, nq, nq> coriolis = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Gravity vector.
        Eigen::Matrix<Scalar, nq, 1> gravity;

        /// @brief Input mapping matrix.
        Eigen::Matrix<Scalar, nq, nq> input_mapping = Eigen::Matrix<Scalar, nq, nq>::Identity();

        /// @brief Damping matrix.
        Eigen::Matrix<Scalar, nq, nq> damping = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Kinetic energy.
        Scalar kinetic_energy = 0;

        /// @brief Potential energy.
        Scalar potential_energy = 0;

        /// @brief Total_energy
        Scalar total_energy = 0;

        /// @brief Vector of forward kinematics Data.
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics = {};

        /// @brief Spatial transforms from parent to child links.
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xup =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Motion subspace matrices for the joints.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> S =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial velocities of the robot links.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> v =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial acceleration bias terms for the robot links.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> c =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Articulated-body inertia matrices for the robot links.
        std::vector<Eigen::Matrix<Scalar, 6, 6>> IA =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Articulated-body forces for the robot links.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> pA =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial force projections for the joints.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> U =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Joint force inertia terms for the robot links.
        std::vector<Scalar> d = std::vector<Scalar>(nq, 0);

        /// @brief Joint force bias terms for the robot links.
        std::vector<Scalar> u = std::vector<Scalar>(nq, 0);

        /// @brief Spatial accelerations of the robot links.
        std::vector<Eigen::Matrix<Scalar, 6, 1>> a =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        Eigen::Transform<Scalar, 3, Eigen::Isometry> T = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 1>> avp =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 1>> fvp =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        Eigen::Matrix<Scalar, nq, 1> C = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// @brief
        Eigen::Matrix<Scalar, 6, 1> fh = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 6>> IC =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());


        /// @brief Gravity vector in spatial coordinates.
        Eigen::Matrix<Scalar, 6, 1> spatial_gravity = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /**
         * @brief Casts the data to a new scalar type.
         * @tparam NewScalar scalar type to cast the data to.
         * @return Data with new scalar type.
         */
        template <typename NewScalar>
        Data<NewScalar, nq> cast() {
            Data<NewScalar, nq> new_res;
            new_res.q                = q.template cast<NewScalar>();
            new_res.dq               = dq.template cast<NewScalar>();
            new_res.ddq              = ddq.template cast<NewScalar>();
            new_res.tau              = tau.template cast<NewScalar>();
            new_res.mass_matrix      = mass_matrix.template cast<NewScalar>();
            new_res.inv_mass_matrix  = inv_mass_matrix.template cast<NewScalar>();
            new_res.coriolis         = coriolis.template cast<NewScalar>();
            new_res.gravity          = gravity.template cast<NewScalar>();
            new_res.input_mapping    = input_mapping.template cast<NewScalar>();
            new_res.damping          = damping.template cast<NewScalar>();
            new_res.kinetic_energy   = NewScalar(kinetic_energy);
            new_res.potential_energy = NewScalar(potential_energy);
            new_res.total_energy     = NewScalar(total_energy);
            for (int i = 0; i < forward_kinematics.size(); i++) {
                new_res.forward_kinematics[i] = forward_kinematics[i].template cast<NewScalar>();
            }
            for (int i = 0; i < Xup.size(); i++) {
                new_res.Xup[i] = Xup[i].template cast<NewScalar>();
                new_res.S[i]   = S[i].template cast<NewScalar>();
                new_res.v[i]   = v[i].template cast<NewScalar>();
                new_res.c[i]   = c[i].template cast<NewScalar>();
                new_res.IA[i]  = IA[i].template cast<NewScalar>();
                new_res.pA[i]  = pA[i].template cast<NewScalar>();
                new_res.U[i]   = U[i].template cast<NewScalar>();
                new_res.d[i]   = NewScalar(d[i]);
                new_res.u[i]   = NewScalar(u[i]);
                new_res.a[i]   = a[i].template cast<NewScalar>();
                new_res.avp[i] = avp[i].template cast<NewScalar>();
                new_res.fvp[i] = fvp[i].template cast<NewScalar>();
                new_res.IC[i]  = IC[i].template cast<NewScalar>();
            }
            new_res.C               = C.template cast<NewScalar>();
            new_res.fh              = fh.template cast<NewScalar>();
            new_res.spatial_gravity = spatial_gravity.template cast<NewScalar>();
            return new_res;
        }
    };
}  // namespace tinyrobotics

#endif