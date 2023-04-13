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
        Eigen::Matrix<Scalar, nv, 1> eigen_grad;

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
        /// @brief Use gradient descent to solve inverse kinematics.
        JACOBIAN,

        /// @brief Use NLopt to solve inverse kinematics.
        NLOPT,

        /// @brief Use NLopt to solve inverse kinematics with autodiff gradient.
        NLOPT_AUTODIFF,

        /// @brief Use Levenberg-Marquardt to solve inverse kinematics.
        LEVENBERG_MARQUARDT
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

        /// @brief Weighting matrix for joint change
        Eigen::Matrix<Scalar, nq, nq> W = 1e-3 * Eigen::Matrix<Scalar, nq, nq>::Identity();

        // ****************** Levenberg-Marquardt Method options ******************
        /// @brief Inital damping factor for Levenberg-Marquardt
        Scalar initial_damping = 1e-2;

        /// @brief Damping decrease factor for Levenberg-Marquardt
        Scalar damping_decrease_factor = 1e-1;

        /// @brief Damping increase factor for Levenberg-Marquardt
        Scalar damping_increase_factor = 2;
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

        // Stack the position and orientation errors
        Eigen::Matrix<Scalar, 6, 1> pose_error;
        pose_error.head(3) << current_pose.translation() - desired_pose.translation();
        pose_error.tail(3) << current_pose.linear().eulerAngles(0, 1, 2) - desired_pose.linear().eulerAngles(0, 1, 2);

        // Compute the cost: q^T*W*q + (k(q) - x*)^TK*(k(q) - x*))), where k(q) is the current pose, x* is the desired
        // pose, and W and K are the weighting matrices
        Eigen::Matrix<Scalar, 1, 1> cost =
            0.5 * pose_error.transpose() * options.K * pose_error + 0.5 * (q - q0).transpose() * options.W * (q - q0);

        // Compute the gradient of the cost function TODO: Fix this
        Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q, target_link_name);
        gradient                       = J.transpose() * options.K * pose_error + options.W * (q - q0);

        return cost(0);
    }


    /**
     * @brief Cost function for general inverse kinematics with autodiff jacobian.
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
    Scalar cost_autodiff(const Eigen::Matrix<Scalar, nq, 1>& q,
                         Model<Scalar, nq>& model,
                         const std::string& target_link_name,
                         const std::string& source_link_name,
                         const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                         const Eigen::Matrix<Scalar, nq, 1>& q0,
                         Eigen::Matrix<Scalar, nq, 1>& gradient,
                         const InverseKinematicsOptions<Scalar, nq>& options) {

        using ADScalar = Eigen::AutoDiffScalar<Eigen::Matrix<Scalar, nq, 1>>;

        // Cast model to Eigen::AutoDiffScalar
        Model<ADScalar, nq> model_ad = model.template cast<ADScalar>();

        // Initialize q_ad with derivatives
        Eigen::Matrix<ADScalar, nq, 1> q_ad;
        Eigen::Matrix<ADScalar, nq, 1> q0_ad;
        for (int i = 0; i < nq; ++i) {
            q_ad[i]                  = ADScalar(q[i]);
            q_ad[i].derivatives()[i] = 1.0;
            q0_ad[i]                 = ADScalar(q0[i]);
        }

        // Compute the current pose
        Eigen::Transform<ADScalar, 3, Eigen::Isometry> current_pose =
            forward_kinematics(model_ad, q_ad, target_link_name, source_link_name);

        // Stack the position and orientation errors
        Eigen::Matrix<ADScalar, 6, 1> pose_error;
        pose_error.head(3) << current_pose.translation() - desired_pose.translation();
        pose_error.tail(3) << current_pose.linear().eulerAngles(0, 1, 2) - desired_pose.linear().eulerAngles(0, 1, 2);

        Eigen::Matrix<ADScalar, 6, 6> K_ad   = options.K.template cast<ADScalar>();
        Eigen::Matrix<ADScalar, nq, nq> W_ad = options.W.template cast<ADScalar>();

        // Compute the cost: q^T*W*q + (k(q) - x*)^TK*(k(q) - x*))), where k(q) is the current pose, x* is the desired
        // pose, and W and K are the weighting matrices
        Eigen::Matrix<ADScalar, 1, 1> cost =
            pose_error.transpose() * K_ad * pose_error + (q_ad - q0_ad).transpose() * W_ad * (q_ad - q0_ad);

        // Extract the gradient from the gradient_ad
        for (int i = 0; i < nq; ++i) {
            gradient[i] = cost[0].derivatives()[i];
        }

        // Return the value of the cost function
        return Scalar(cost(0).value());
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
            if (options.method == InverseKinematicsMethod::NLOPT_AUTODIFF) {
                cost_value =
                    cost_autodiff(q, model, target_link_name, source_link_name, desired_pose, q0, gradient, options);
            }
            else {
                cost_value = cost(q, model, target_link_name, source_link_name, desired_pose, q0, gradient, options);
            }

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
            Eigen::Matrix<Scalar, 6, 1> pose_error;
            pose_error.head(3) = current_pose.translation() - desired_pose.translation();
            pose_error.tail(3) =
                current_pose.linear().eulerAngles(0, 1, 2) - desired_pose.linear().eulerAngles(0, 1, 2);

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
            Eigen::Matrix<Scalar, 6, 1> pose_error;
            pose_error.head(3) = current_pose.translation() - desired_pose.translation();
            pose_error.tail(3) << current_pose.linear().eulerAngles(0, 1, 2)
                                      - desired_pose.linear().eulerAngles(0, 1, 2);

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
            case InverseKinematicsMethod::NLOPT_AUTODIFF:
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
            default: throw std::runtime_error("Unknown inverse kinematics method");
        }
    }
}  // namespace tinyrobotics
#endif
