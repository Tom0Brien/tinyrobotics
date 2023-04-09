#ifndef TR_INVERSEKINEMATICS_HPP
#define TR_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/AutoDiff>

#include "fmin.hpp"
#include "kinematics.hpp"
#include "math.hpp"
#include "model.hpp"

/** \file inversekinematics.hpp
 * @brief Contains inverse kinematics algorithms.
 */
namespace tinyrobotics {

    // /**
    //  * @brief The cost function.
    //  * @param model The robot model.
    //  * @param q The joint configuration of the robot.
    //  * @param target_link_name {t} The link to which the transform is computed.
    //  * @param source_link_name {s} The link from which the transform is computed.
    //  * @return The configuration vector of the robot model which achieves the
    //  * desired pose.
    //  */
    // template <typename Scalar, int nq>
    // Scalar cost(const Eigen::Matrix<Scalar, nq, 1>& q,
    //             const Model<Scalar, nq>& model,
    //             const std::string& target_link_name,
    //             const std::string& source_link_name,
    //             const Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst_desired,
    //             const Eigen::Matrix<Scalar, nq, 1>& q0) {

    //     // Compute the current pose
    //     Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst_current =
    //         forward_kinematics(model, q, target_link_name, source_link_name);

    //     // Compute the translation error
    //     Eigen::Matrix<Scalar, 3, 1> t_err = Hst_current.translation() - Hst_desired.translation();

    //     // Compute the orientation error
    //     Eigen::Matrix<Scalar, 3, 3> Rst_current = Hst_current.linear();
    //     Eigen::Matrix<Scalar, 3, 3> Rst_desired = Hst_desired.linear();
    //     Eigen::Matrix<Scalar, 1, 1> o_err       = Eigen::Matrix<Scalar, 1, 1>(
    //         (Eigen::Matrix<Scalar, 3, 3>::Identity() - Rst_desired * Rst_current.transpose()).diagonal().sum());

    //     // Error between the initial guess and the current configuration
    //     Eigen::Matrix<Scalar, nq, 1> q_err = q - q0;

    //     // Quadratic cost function q^T*W*q + (k(q) - x*)^TK*(l(q) - x*)))
    //     Eigen::Matrix<Scalar, 3, 3> K   = 1e2 * Eigen::Matrix<Scalar, 3, 3>::Identity();
    //     Eigen::Matrix<Scalar, nq, nq> W = 1e-1 * Eigen::Matrix<Scalar, nq, nq>::Identity(model.n_q, model.n_q);
    //     Eigen::Matrix<Scalar, 1, 1> cost =
    //         t_err.transpose() * K * t_err + q_err.transpose() * W * q_err + o_err.transpose() * 1e3 * o_err;

    //     return cost(0, 0);
    // }

    template <typename Scalar, int nq>
    Scalar cost(const Eigen::Matrix<Scalar, nq, 1>& q,
                Model<Scalar, nq>& model,
                const std::string& target_link_name,
                const std::string& source_link_name,
                const Eigen::Transform<Scalar, 3, Eigen::Isometry>& Hst_desired,
                const Eigen::Matrix<Scalar, nq, 1>& q0,
                Eigen::Matrix<Scalar, nq, 1>& gradient) {

        // Compute the current pose
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst_current =
            forward_kinematics(model, q, target_link_name, source_link_name);

        // Compute the position error
        Eigen::Matrix<Scalar, 3, 1> position_error = Hst_current.translation() - Hst_desired.translation();

        // Compute the orientation error
        Eigen::Matrix<Scalar, 3, 1> orientation_error =
            rotation_to_rpy(Hst_current.linear()) - rotation_to_rpy(Hst_desired.linear());

        // Stack the position and orientation errors
        Eigen::Matrix<Scalar, 6, 1> pose_error;
        pose_error << position_error, orientation_error;

        // Compute the cost
        Eigen::Matrix<Scalar, 1, 1> cost = pose_error.transpose() * pose_error + (q - q0).transpose() * (q - q0);

        // std::cout << "cost: " << cost(0, 0) << std::endl;

        // Compute the gradient of the cost function
        Eigen::Matrix<Scalar, 6, nq> Ja = analytical_jacobian(model, q, target_link_name);

        gradient = 2 * Ja.transpose() * pose_error + 2 * (q - q0);

        return cost(0, 0);
    }

    // template <typename Scalar, int nq>
    // Eigen::Matrix<Scalar, nq, 1> cost_gradient(const Eigen::Matrix<Scalar, nq, 1>& q,
    //                                            const Model<Scalar, nq>& model,
    //                                            const std::string& target_link_name,
    //                                            const std::string& source_link_name,
    //                                            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& Hst_desired) {

    //     // Compute the position error
    //     Eigen::Matrix<Scalar, 3, 1> position_error =
    //         translation(model, q, target_link_name, source_link_name) - T_desired.translation();

    //     std::cout << "position_error: " << position_error.transpose() << std::endl;

    //     // Compute the gradient of the cost function
    //     Eigen::Matrix<Scalar, 6, nq> J         = geometric_jacobian(model, q, target_link_name);
    //     Eigen::Matrix<Scalar, 3, nq> Jv        = J.topRows(3);
    //     Eigen::Matrix<Scalar, nq, 1> grad_cost = Jv.transpose() * position_error;
    //     return grad_cost;
    // }

    // template <typename Scalar, int nq>
    // Eigen::Matrix<Scalar, 6, nq> geometric_jacobian_autodiff(Model<Scalar, nq>& model,
    //                                                          const Eigen::Matrix<Scalar, nq, 1>& q,
    //                                                          const std::string& target_link_name) {
    //     using ADScalar = Eigen::AutoDiffScalar<Eigen::Matrix<Scalar, nq, 1>>;

    //     // Cast model to Eigen::AutoDiffScalar
    //     Model<ADScalar, nq> model_ad = model.template cast<ADScalar>();

    //     // Cast configuration to Eigen::AutoDiffScalar
    //     Eigen::Matrix<ADScalar, nq, 1> q_ad;
    //     for (int i = 0; i < nq; ++i) {
    //         q_ad[i]                  = q[i];
    //         q_ad[i].derivatives()[i] = 1.0;
    //     }

    //     // Compute the forward kinematics using autodiff
    //     Eigen::Matrix<ADScalar, 3, 1> fk = translation(model_ad, q_ad, target_link_name);

    //     // Extract the Jacobian from the derivatives
    //     Eigen::Matrix<Scalar, 6, nq> J;
    //     J.setZero();
    //     for (int i = 0; i < 3; ++i) {
    //         for (int j = 0; j < nq; ++j) {
    //             J(i, j) = fk[i].derivatives()[j];
    //         }
    //     }

    //     return J;
    // }

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> cost_gradient(const Eigen::Matrix<Scalar, nq, 1>& q,
                                               Model<Scalar, nq>& model,
                                               const std::string& target_link_name,
                                               const std::string& source_link_name,
                                               const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T_desired) {
        // Cast model to Eigen::AutoDiffScalar
        using ADScalar               = Eigen::AutoDiffScalar<Eigen::Matrix<double, nq, 1>>;
        Model<ADScalar, nq> model_ad = model.template cast<ADScalar>();

        // Cast configuration to Eigen::AutoDiffScalar
        Eigen::Matrix<ADScalar, nq, 1> q_ad;
        for (int i = 0; i < nq; ++i) {
            q_ad[i]                  = q[i];
            q_ad[i].derivatives()[i] = 0.0;
        }

        // Compute the cost
        Eigen::Transform<ADScalar, 3, Eigen::Isometry> T_desired_ad;
        T_desired_ad.translation() << ADScalar(T_desired.translation()[0]), ADScalar(T_desired.translation()[1]),
            ADScalar(T_desired.translation()[2]);
        ADScalar cost_ad = cost(q_ad, model_ad, target_link_name, source_link_name, T_desired_ad);

        // Extract the gradient from the cost's derivatives
        Eigen::Matrix<Scalar, nq, 1> grad_cost;
        for (int i = 0; i < nq; ++i) {
            grad_cost[i] = cost_ad.derivatives()[i];
        }

        return grad_cost;
    }

    /**
     * @brief Solves the inverse kinematics problem between two links.
     * @param model The robot model.
     * @param target_link_name {t} The link to which the transform is computed.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param desired_pose {d} The desired pose of the target link in the source
     * link frame.
     * @param q0 The initial guess for the configuration vector.
     * @return The configuration vector of the robot model which achieves the
     * desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics(Model<Scalar, nq>& model,
                                                    std::string& target_link_name,
                                                    std::string& source_link_name,
                                                    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                                                    Eigen::Matrix<Scalar, nq, 1> q0) {
        // Create the NLopt optimizer and set the algorithm
        nlopt::opt opt(nlopt::LN_COBYLA, nq);
        // Set the objective function
        ObjectiveFunction<Scalar, nq> obj_fun =
            [&](const Eigen::Matrix<Scalar, nq, 1>& q, Eigen::Matrix<Scalar, nq, 1>& grad, void* data) -> Scalar {
            (void) data;  // Unused in this case

            // Compute the cost
            Eigen::Matrix<Scalar, nq, 1> gradient;
            Scalar cost_value = cost(q, model, target_link_name, source_link_name, desired_pose, q0, gradient);

            // Pass the gradient to NLopt
            // eigen_to_nlopt<Scalar, nq>(gradient, grad);
            for (int i = 0; i < nq; ++i) {
                grad[i] = gradient(i);
            }

            return cost_value;
        };
        opt.set_min_objective(eigen_objective_wrapper<Scalar, nq>, &obj_fun);

        // Set the optimization tolerances
        opt.set_xtol_rel(1e-12);
        opt.set_ftol_rel(1e-12);

        // Convert the initial guess to NLopt format
        std::vector<Scalar> x_initial(nq);
        eigen_to_nlopt<Scalar, nq>(q0, x_initial);

        // Optimize
        Scalar minf;
        nlopt::result result = opt.optimize(x_initial, minf);

        // Convert the optimized solution back to an Eigen vector
        Eigen::Matrix<Scalar, nq, 1> optimized_solution(nq);
        nlopt_to_eigen<Scalar, nq>(x_initial, optimized_solution);

        // Return the optimized solution
        return optimized_solution;
    }


}  // namespace tinyrobotics
#endif
