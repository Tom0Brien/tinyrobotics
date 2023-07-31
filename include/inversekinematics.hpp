#ifndef TR_INVERSEKINEMATICS_HPP
#define TR_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlopt.hpp>
#include <unsupported/Eigen/AutoDiff>

#include "math.hpp"

namespace tinyrobotics {

    /**
     * @brief Converts an Eigen vector to an std::vector.
     * @param eigen_vec The Eigen vector to convert.
     * @param nlopt_vec The resulting std::vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void eigen_to_nlopt(const Eigen::Matrix<Scalar, n, 1>& eigen_vec, std::vector<Scalar>& nlopt_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        Eigen::Map<Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the Eigen vector to the std::vector view
        nlopt_eigen_vec = eigen_vec;
    }

    /**
     * @brief Converts an std::vector to an Eigen vector.
     * @param nlopt_vec The std::vector to convert.
     * @param eigen_vec The resulting Eigen vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void nlopt_to_eigen(const std::vector<Scalar>& nlopt_vec, Eigen::Matrix<Scalar, n, 1>& eigen_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        const Eigen::Map<const Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the std::vector view to the Eigen vector
        eigen_vec = nlopt_eigen_vec;
    }

    /**
     * @brief Type definition for an objective function that takes an Eigen vector as input and returns a scalar value.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     */
    template <typename Scalar, int nv>
    using ObjectiveFunction =
        std::function<Scalar(const Eigen::Matrix<Scalar, nv, 1>&, Eigen::Matrix<Scalar, nv, 1>&, void*)>;

    /**
     * @brief Wrapper function that converts input and output between NLopt and Eigen formats for an objective function.
     * @param n The size of the input vector.
     * @param x The input vector in NLopt format.
     * @param grad The gradient vector in NLopt format.
     * @param data Pointer to additional data that is passed to the objective function.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     * @return The scalar value of the objective function.
     */
    template <typename Scalar, int nv>
    inline Scalar eigen_objective_wrapper(unsigned n, const Scalar* x, Scalar* grad, void* data) {
        ObjectiveFunction<Scalar, nv>& obj_fun = *static_cast<ObjectiveFunction<Scalar, nv>*>(data);

        // Convert input from NLopt format to Eigen format
        Eigen::Map<const Eigen::Matrix<Scalar, nv, 1>> eigen_x(x, n);
        Eigen::Matrix<Scalar, nv, 1> eigen_grad = Eigen::Matrix<Scalar, nv, 1>::Zero();

        if (grad) {
            eigen_grad.resize(n);
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        // Call the actual objective function implemented with Eigen
        Scalar result = obj_fun(eigen_x, eigen_grad, data);

        if (grad) {
            // Copy the gradient back to NLopt format
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        return result;
    }

    /// @brief Solver methods for inverse kinematics.
    enum class InverseKinematicsMethod {
        /// @brief Jacobian method.
        JACOBIAN,

        /// @brief NLopt method.
        NLOPT,

        /// @brief Levenberg-Marquardt method.
        LEVENBERG_MARQUARDT,

        /// @brief Particle swarm optimization method.
        PARTICLE_SWARM,

        /// @brief BFGS method.
        BFGS
    };

    /// @brief Options for inverse kinematics solver.
    template <typename Scalar, int nq>
    struct InverseKinematicsOptions {
        /// @brief Relative tolerance for cost function value
        Scalar tolerance = 1e-6;

        /// @brief Relative tolerance for NLopt optimization variables
        Scalar xtol_rel = 1e-12;

        /// @brief Maximum number of iterations for optimization
        int max_iterations = 5e3;

        /// @brief Inverse kinematics solver method
        InverseKinematicsMethod method = InverseKinematicsMethod::JACOBIAN;

        // ****************** Jacobian Method options ******************
        /// @brief Step size
        Scalar step_size = 1e-1;

        // ****************** NLopt Method options ******************
        /// @brief NLopt optimization algorithm to use
        nlopt::algorithm algorithm = nlopt::LD_SLSQP;

        /// @brief Weighting matrix for pose error
        Eigen::Matrix<Scalar, 6, 6> K = Eigen::Matrix<Scalar, 6, 6>::Identity();

        /// @brief Weighting matrix for joint displacement from initial configuration
        Eigen::Matrix<Scalar, nq, nq> W = 1e-3 * Eigen::Matrix<Scalar, nq, nq>::Identity();

        // ****************** Levenberg-Marquardt Method options ******************
        /// @brief Inital damping factor for Levenberg-Marquardt
        Scalar initial_damping = 1e-2;

        /// @brief Damping decrease factor for Levenberg-Marquardt
        Scalar damping_decrease_factor = 1e-1;

        /// @brief Damping increase factor for Levenberg-Marquardt
        Scalar damping_increase_factor = 2;

        // ****************** Particle Swarm Optimization Method options ******************
        /// @brief Number of particles in the swarm
        int num_particles = 5e2;

        /// @brief Inertia weight (omega) for updating particle velocities
        Scalar omega = 1e-1;

        /// @brief Cognitive weight (c1) for updating particle velocities
        Scalar c1 = 1e-1;

        /// @brief Social weight (c2) for updating particle velocities
        Scalar c2 = 1e-1;

        /// @brief Scale for initializing particle positions around the initial guess
        Scalar init_position_scale = 1e-1;
    };
}  // namespace tinyrobotics
#endif
