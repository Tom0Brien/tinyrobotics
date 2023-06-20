#ifndef TR_INVERSEKINEMATICS_HPP
#define TR_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlopt.hpp>
#include <unsupported/Eigen/AutoDiff>

#include "kinematics.hpp"
#include "math.hpp"
#include "model.hpp"

/** \file inversekinematics.hpp
 * @brief Contains inverse kinematics algorithms.
 */
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

    /**
     * @brief Cost function for general inverse kinematics with analytical jacobian.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 Initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Scalar cost(const Eigen::Matrix<Scalar, nq, 1>& q,
                Model<Scalar, nq>& model,
                const std::string& target_link_name,
                const std::string& source_link_name,
                const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                const Eigen::Matrix<Scalar, nq, 1>& q0,
                Eigen::Matrix<Scalar, nq, 1>& gradient,
                const InverseKinematicsOptions<Scalar, nq>& options) {

        // Compute the current pose
        Eigen::Transform<Scalar, 3, Eigen::Isometry> current_pose =
            forward_kinematics(model, q, target_link_name, source_link_name);

        // Compute the pose error
        Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneous_error(current_pose, desired_pose);

        // Compute the cost: q^T*W*q + (k(q) - x*)^TK*(k(q) - x*))), where k(q) is the current pose, x* is the desired
        // pose, and W and K are the weighting matrices
        Eigen::Matrix<Scalar, 1, 1> cost =
            0.5 * pose_error.transpose() * options.K * pose_error + 0.5 * (q - q0).transpose() * options.W * (q - q0);

        // Compute the gradient of the cost function
        Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q, target_link_name);
        gradient                       = J.transpose() * options.K * pose_error + options.W * (q - q0);

        return cost(0);
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using NLopt.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics_nlopt(
        Model<Scalar, nq>& model,
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {
        // Create the NLopt optimizer and set the algorithm
        nlopt::opt opt(options.algorithm, nq);

        // Wrap the objective function in a lambda function
        ObjectiveFunction<Scalar, nq> obj_fun =
            [&](const Eigen::Matrix<Scalar, nq, 1>& q, Eigen::Matrix<Scalar, nq, 1>& grad, void* data) -> Scalar {
            (void) data;  // Unused in this case

            // Compute the cost and gradient
            Eigen::Matrix<Scalar, nq, 1> gradient;
            Scalar cost_value = 0.0;
            cost_value        = cost(q, model, target_link_name, source_link_name, desired_pose, q0, gradient, options);

            // Pass the gradient to NLopt
            for (int i = 0; i < nq; ++i) {
                grad[i] = gradient(i);
            }

            return cost_value;
        };
        // Set the objective function
        opt.set_min_objective(eigen_objective_wrapper<Scalar, nq>, &obj_fun);

        // Set the optimization tolerances
        opt.set_xtol_rel(options.xtol_rel);
        opt.set_ftol_rel(options.tolerance);

        // Set the maximum number of iterations
        opt.set_maxeval(options.max_iterations);

        // Convert the initial guess to NLopt format
        std::vector<Scalar> x_initial(nq);
        eigen_to_nlopt<Scalar, nq>(q0, x_initial);

        // Find the optimal solution
        Scalar minf;
        opt.optimize(x_initial, minf);

        // Convert the optimized solution back to an Eigen
        Eigen::Matrix<Scalar, nq, 1> optimized_solution(nq);
        nlopt_to_eigen<Scalar, nq>(x_initial, optimized_solution);
        return optimized_solution;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using the Jacobian method.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics_jacobian(
        Model<Scalar, nq>& model,
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {

        // Set the initial guess
        Eigen::Matrix<Scalar, nq, 1> q_current = q0;

        // Iterate until the maximum number of iterations is reached
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {

            // Compute the current pose
            Eigen::Transform<Scalar, 3, Eigen::Isometry> current_pose =
                forward_kinematics(model, q_current, target_link_name, source_link_name);

            // Compute the pose error vector
            Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneous_error(current_pose, desired_pose);

            // Check if the error is within tolerance
            if (pose_error.norm() < options.tolerance) {
                break;
            }

            // Compute the Jacobian matrix
            Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q_current, target_link_name);

            // Compute the change in configuration
            Eigen::Matrix<Scalar, nq, 1> delta_q =
                J.completeOrthogonalDecomposition().pseudoInverse() * (-options.step_size * pose_error);

            // Update the current configuration
            q_current += delta_q;
        }

        // Return the final configuration
        return q_current;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using the Levenberg-Marquardt method.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics_levenberg_marquardt(
        Model<Scalar, nq>& model,
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {

        // Set the initial guess
        Eigen::Matrix<Scalar, nq, 1> q_current = q0;

        // Initialize the damping factor
        Scalar lambda = options.initial_damping;

        // Iterate until the maximum number of iterations is reached
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {

            // Compute the current pose
            Eigen::Transform<Scalar, 3, Eigen::Isometry> current_pose =
                forward_kinematics(model, q_current, target_link_name, source_link_name);

            // Compute the pose error vector
            Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneous_error(current_pose, desired_pose);

            // Check if the error is within tolerance
            if (pose_error.norm() < options.tolerance) {
                break;
            }

            // Compute the Jacobian matrix
            Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q_current, target_link_name);

            // Compute the Hessian approximation and the gradient
            Eigen::Matrix<Scalar, nq, nq> H = J.transpose() * J;
            Eigen::Matrix<Scalar, nq, 1> g  = J.transpose() * pose_error;

            // Levenberg-Marquardt update
            Eigen::Matrix<Scalar, nq, 1> delta_q =
                (H + lambda * Eigen::Matrix<Scalar, nq, nq>(H.diagonal().asDiagonal())).ldlt().solve(-g);

            // Test the new configuration
            Eigen::Matrix<Scalar, nq, 1> q_new = q_current + delta_q;
            Eigen::Transform<Scalar, 3, Eigen::Isometry> new_pose =
                forward_kinematics(model, q_new, target_link_name, source_link_name);

            // Compute the new error vector
            Eigen::Matrix<Scalar, 6, 1> new_pose_error;
            new_pose_error.head(3) = new_pose.translation() - desired_pose.translation();
            Eigen::AngleAxis<Scalar> new_rot_error(new_pose.rotation().transpose() * desired_pose.rotation());
            new_pose_error.tail(3) = new_rot_error.angle() * new_rot_error.axis();

            // Check if the new error is smaller than the old error
            if (new_pose_error.norm() < pose_error.norm()) {
                // Accept the new configuration
                q_current = q_new;
                lambda *= options.damping_decrease_factor;
            }
            else {
                // Reject the new configuration and increase the damping factor
                lambda *= options.damping_increase_factor;
            }
        }

        // Return the final configuration
        return q_current;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using Particle Swarm Optimization.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics_pso(
        Model<Scalar, nq>& model,
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {
        // Particle Swarm Optimization parameters
        int num_particles = options.num_particles;
        Scalar omega      = options.omega;
        Scalar c1         = options.c1;
        Scalar c2         = options.c2;

        // Initialize particles and velocities
        std::vector<Eigen::Matrix<Scalar, nq, 1>> particles(num_particles, q0);
        std::vector<Eigen::Matrix<Scalar, nq, 1>> velocities(num_particles, Eigen::Matrix<Scalar, nq, 1>::Zero());
        std::vector<Scalar> fitness_values(num_particles, std::numeric_limits<Scalar>::max());
        Eigen::Matrix<Scalar, nq, 1> best_global_position = q0;
        Scalar best_global_fitness                        = std::numeric_limits<Scalar>::max();

        // Initialize particles
        for (int i = 0; i < num_particles; ++i) {
            particles[i] = q0 + options.init_position_scale * Eigen::Matrix<Scalar, nq, 1>::Random();
        }

        // Main optimization loop
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            for (int i = 0; i < num_particles; ++i) {
                // Evaluate the fitness of the particle
                Eigen::Transform<Scalar, 3, Eigen::Isometry> current_pose =
                    forward_kinematics(model, particles[i], target_link_name, source_link_name);
                Scalar fitness_value = homogeneous_error(current_pose, desired_pose).squaredNorm();

                // Update the best position of the particle
                if (fitness_value < fitness_values[i]) {
                    fitness_values[i] = fitness_value;
                }

                // Update the best global position
                if (fitness_value < best_global_fitness) {
                    best_global_fitness  = fitness_value;
                    best_global_position = particles[i];
                }
            }

            // Update the particles' velocities and positions
            for (int i = 0; i < num_particles; ++i) {
                // Update the velocity
                velocities[i] =
                    omega * velocities[i]
                    + c1 * Eigen::Matrix<Scalar, nq, 1>::Random().cwiseProduct(particles[i] - best_global_position)
                    + c2 * Eigen::Matrix<Scalar, nq, 1>::Random().cwiseProduct(best_global_position - particles[i]);

                // Update the position
                particles[i] += velocities[i];
            }

            // Check if the error is within tolerance
            if (best_global_fitness < options.tolerance) {
                break;
            }
        }

        // Return the best global position
        return best_global_position;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using Broyden-Fletcher-Goldfarb-Shanno (BFGS).
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics_bfgs(
        Model<Scalar, nq>& model,
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {

        Eigen::Matrix<Scalar, nq, 1> q = q0;
        Eigen::Matrix<Scalar, nq, 1> grad;
        Eigen::Matrix<Scalar, nq, nq> inverse_hessian = Eigen::Matrix<Scalar, nq, nq>::Identity();

        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            // Compute the gradient and cost using the provided cost function
            Scalar cost_value = cost(q, model, target_link_name, source_link_name, desired_pose, q0, grad, options);

            if (grad.norm() < options.tolerance) {
                break;
            }

            Eigen::Matrix<Scalar, nq, 1> direction = -inverse_hessian * grad;
            Scalar alpha                           = 1.0;
            Eigen::Matrix<Scalar, nq, 1> grad_new;
            Scalar cost_new;
            // Line search with backtracking
            do {
                alpha *= 0.5;
                Eigen::Matrix<Scalar, nq, 1> q_new = q + alpha * direction;
                cost_new = cost(q_new, model, target_link_name, source_link_name, desired_pose, q0, grad_new, options);
            } while (cost_new > cost_value + options.step_size * alpha * grad.dot(direction));

            Eigen::Matrix<Scalar, nq, 1> q_new = q + alpha * direction;

            Eigen::Matrix<Scalar, nq, 1> s = q_new - q;
            Eigen::Matrix<Scalar, nq, 1> y = grad_new - grad;
            Scalar rho                     = 1.0 / y.dot(s);

            Eigen::Matrix<Scalar, nq, nq> I = Eigen::Matrix<Scalar, nq, nq>::Identity();
            inverse_hessian = (I - rho * s * y.transpose()) * inverse_hessian * (I - rho * y * s.transpose())
                              + rho * s * s.transpose();

            q    = q_new;
            grad = grad_new;
        }

        return q;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links using user specified method.
     * @param model tinyrobotics model.
     * @param target_link_name {t} Link to which the transform is computed.
     * @param source_link_name {s} Link from which the transform is computed.
     * @param desired_pose Desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics(Model<Scalar, nq>& model,
                                                    const std::string& target_link_name,
                                                    const std::string& source_link_name,
                                                    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                                                    const Eigen::Matrix<Scalar, nq, 1> q0,
                                                    const InverseKinematicsOptions<Scalar, nq>& options) {
        switch (options.method) {
            case InverseKinematicsMethod::NLOPT:
                return inverse_kinematics_nlopt(model, target_link_name, source_link_name, desired_pose, q0, options);
            case InverseKinematicsMethod::JACOBIAN:
                return inverse_kinematics_jacobian(model,
                                                   target_link_name,
                                                   source_link_name,
                                                   desired_pose,
                                                   q0,
                                                   options);
            case InverseKinematicsMethod::LEVENBERG_MARQUARDT:
                return inverse_kinematics_levenberg_marquardt(model,
                                                              target_link_name,
                                                              source_link_name,
                                                              desired_pose,
                                                              q0,
                                                              options);
            case InverseKinematicsMethod::PARTICLE_SWARM:
                return inverse_kinematics_pso(model, target_link_name, source_link_name, desired_pose, q0, options);
            case InverseKinematicsMethod::BFGS:
                return inverse_kinematics_bfgs(model, target_link_name, source_link_name, desired_pose, q0, options);
            default: throw std::runtime_error("Unknown inverse kinematics method");
        }
    }
}  // namespace tinyrobotics
#endif
