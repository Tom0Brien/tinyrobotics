#ifndef RML_INVERSEKINEMATICS_HPP
#define RML_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include "Kinematics.hpp"
#include "Model.hpp"

namespace RML {

    /**
     * @brief Defines the variable set for the IK problem, which is the configuration for the robot.
     * @param model The robot model.
     */
    template <typename Scalar, int nq>
    class IKVariables : public ifopt::VariableSet {
    public:
        /**
         * @brief Every variable set has a name, here "var_set1". this allows the constraints
         * and costs to define values and Jacobians specifically w.r.t this variable set.
         */
        IKVariables(const std::string& name, Model<Scalar, nq>& _model, Eigen::Matrix<Scalar, nq, 1> q0)
            : VariableSet(_model.n_q, name) {
            // the initial values where the NLP starts iterating from
            q.resize(_model.n_q);
            q = q0;
        }

        /**
         * @brief Overrides the variable set
         */
        void SetVariables(const ifopt::Component::VectorXd& qin) override {
            q = qin;
        };

        /**
         * @brief Gets the configuration vector
         * @return The configuration vector
         */
        ifopt::Component::VectorXd GetValues() const override {
            return q;
        };

        /**
         * @brief Each variable has an upper and lower bound set here
         * @return The bounds
         */
        ifopt::Component::VecBound GetBounds() const override {
            ifopt::Component::VecBound bounds(GetRows());
            for (int i = 0; i < GetRows(); i++) {
                bounds.at(i) = {-2 * M_PI, 2 * M_PI};
            }
            return bounds;
        }

    private:
        /// @brief The variable set, which is the robots configuration vector
        Eigen::Matrix<Scalar, nq, 1> q;
    };

    /**
     * @brief Defines the constraints for the IK problem (not currently used)
     * @param
     */
    template <typename Scalar, int nq>
    class IKConstraint : public ifopt::ConstraintSet {
    public:
        IKConstraint() : IKConstraint("constraint1") {}

        // This constraint set just contains 1 constraint, however generally
        // each set can contain multiple related constraints.
        IKConstraint(const std::string& name) : ConstraintSet(1, name) {}

        // The constraint value minus the constant value "1", moved to bounds.
        VectorXd GetValues() const override {
            VectorXd g(GetRows());
            Eigen::Matrix<Scalar, nq, 1> x = GetVariables()->GetComponent("var_set1")->GetValues();
            g(0)                           = std::pow(x(0), 2) + x(1);
            return g;
        };

        // The only constraint in this set is an equality constraint to 1.
        // Constant values should always be put into GetBounds(), not GetValues().
        // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
        VecBound GetBounds() const override {
            VecBound b(GetRows());
            b.at(0) = ifopt::Bounds(1.0, 1.0);
            return b;
        }

        // This function provides the first derivative of the constraints.
        // In case this is too difficult to write, you can also tell the solvers to
        // approximate the derivatives by finite differences and not overwrite this
        // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
        // Attention: see the parent class function for important information on sparsity pattern.
        void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override {
            // must fill only that submatrix of the overall Jacobian that relates
            // to this constraint and "var_set1". even if more constraints or variables
            // classes are added, this submatrix will always start at row 0 and column 0,
            // thereby being independent from the overall problem.
            if (var_set == "var_set1") {
                Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x = GetVariables()->GetComponent("var_set1")->GetValues();

                jac_block.coeffRef(0, 0) = 2.0 * x(0);  // derivative of first constraint w.r.t x0
                jac_block.coeffRef(0, 1) = 1.0;         // derivative of first constraint w.r.t x1
            }
        }
    };

    /**
     * @brief Defines the cost function for the IK problem
     * @param model The robot model.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @param desired_pose {d} The desired pose of the target link in the source link frame.
     * @return The cost given a configuration vector.
     */
    template <typename Scalar, typename AutoDiffType, int nq>
    class IKCost : public ifopt::CostTerm {
    public:
        IKCost() : IKCost("IK_cost") {}
        IKCost(const std::string& name,
               Model<AutoDiffType, nq>& _model,
               std::string& _source_link_name,
               std::string& _target_link_name,
               const Eigen::Transform<Scalar, 3, Eigen::Affine>& _desired_pose,
               const Eigen::Matrix<Scalar, nq, 1>& _q0)
            : CostTerm(name) {
            model            = _model;
            source_link_name = _source_link_name;
            target_link_name = _target_link_name;
            desired_pose     = _desired_pose;
            q0               = _q0;
        }

        /// @brief The Model used in the IK problem.
        Model<AutoDiffType, nq> model;

        /// @brief The name of the source link.
        std::string source_link_name;

        /// @brief The name of the target link.
        std::string target_link_name;

        /// @brief The desired pose of the target link in the source link frame.
        Eigen::Transform<Scalar, 3, Eigen::Affine> desired_pose;

        /// @brief The initial conditions for the IK solver.
        Eigen::Matrix<Scalar, nq, 1> q0;


        /**
         * @brief The cost function.
         * @param model The robot model.
         * @param q The joint configuration of the robot.
         * @param source_link_name {s} The link from which the transform is computed.
         * @param target_link_name {t} The link to which the transform is computed.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        static inline Eigen::Matrix<AutoDiffType, 1, 1> cost(
            const Eigen::Matrix<AutoDiffType, nq, 1>& q,
            const Model<AutoDiffType, nq>& model,
            const std::string& source_link_name,
            const std::string& target_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Affine> Hst_desired,
            const Eigen::Matrix<AutoDiffType, nq, 1>& q0) {
            // Compute the forward kinematics from the source link to the target link using the current joint angles.
            Eigen::Transform<AutoDiffType, 3, Eigen::Affine> Hst_current =
                forward_kinematics<AutoDiffType, nq>(model, q, source_link_name, target_link_name);

            // Compute the euler angles for current
            Eigen::Matrix<AutoDiffType, 3, 3> Rst_current = Hst_current.linear();
            // Compute the euler angles for desired
            Eigen::Matrix<AutoDiffType, 3, 3> Rst_desired = Hst_desired.linear();
            Eigen::Matrix<AutoDiffType, 3, 3> R_v_r       = Rst_desired * Rst_current.transpose();

            AutoDiffType orientation_error = (Eigen::Matrix<AutoDiffType, 3, 3>::Identity() - R_v_r).diagonal().sum();
            Eigen::Matrix<AutoDiffType, 1, 1> o_error;
            o_error(0, 0) = orientation_error;

            Eigen::Matrix<AutoDiffType, nq, 1> q_diff = q - q0;

            // Quadratic cost function q^T*W*q + (k(q) - x*)^TK*(l(q) - x*)))
            Eigen::Matrix<AutoDiffType, nq, nq> W = Eigen::Matrix<AutoDiffType, nq, nq>::Identity(model.n_q, model.n_q);
            Eigen::Matrix<AutoDiffType, 3, 3> K   = Eigen::Matrix<AutoDiffType, 3, 3>::Identity();

            // Compute the cost function
            Eigen::Matrix<AutoDiffType, 1, 1> cost =
                ((Hst_current.translation() - Hst_desired.translation()).transpose() * 10 * K
                 * (Hst_current.translation() - Hst_desired.translation()))
                + (q - q0).transpose() * 1e-1 * W * (q - q0) + o_error.transpose() * 100 * o_error;

            return cost;
        }

        /**
         * @brief The cost function, which is scalar quadratic cost
         * @return The value of the cost function
         */
        Scalar GetCost() const override {
            // Cast q and model to autodiff type
            Eigen::Matrix<AutoDiffType, nq, 1> q_auto(
                GetVariables()->GetComponent("configuration_vector")->GetValues());

            // Evaluate the cost function
            Eigen::Matrix<AutoDiffType, 1, 1> cost_val =
                cost(q_auto, model, source_link_name, target_link_name, desired_pose, q0);

            return val(cost_val(0, 0));
        };

        /**
         * @brief Sets the gradient of the cost function given a configuration vector.
         */
        void FillJacobianBlock(std::string var_set, Jacobian& jac) const override {
            if (var_set == "configuration_vector") {
                // Cast q and model to autodiff type
                Eigen::Matrix<AutoDiffType, nq, 1> q_auto(
                    GetVariables()->GetComponent("configuration_vector")->GetValues());

                // The output vector F = f(x) evaluated together with Jacobian matrix below
                Eigen::Matrix<AutoDiffType, 1, 1> F;

                // Compute the Jacobian
                Eigen::Matrix<Scalar, 1, nq> J =
                    jacobian(cost,
                             wrt(q_auto),
                             at(q_auto, model, source_link_name, target_link_name, desired_pose, q0),
                             F);

                // Fill the Jacobian block
                jac.resize(J.rows(), J.cols());
                for (int i = 0; i < model.n_q; i++) {
                    jac.coeffRef(0, i) = J(0, i);
                }
            }
        }
    };

    /**
     * @brief Solves the inverse kinematics problem between two links.
     * @param model The robot model.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @param desired_pose {d} The desired pose of the target link in the source link frame.
     * @param q0 The initial guess for the configuration vector.
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> inverse_kinematics(Model<Scalar, nq>& model,
                                                    std::string& source_link_name,
                                                    std::string& target_link_name,
                                                    const Eigen::Transform<Scalar, 3, Eigen::Affine>& desired_pose,
                                                    Eigen::Matrix<Scalar, nq, 1> q0) {

        // Cast model to autodiff type
        auto autodiff_model = model.template cast<autodiff::dual>();
        // 1. Define the problem
        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<IKVariables<double, nq>>("configuration_vector", model, q0));
        // nlp.AddConstraintSet(std::make_shared<IKConstraint<double>>());
        nlp.AddCostSet(std::make_shared<IKCost<double, autodiff::dual, nq>>("IK_cost",
                                                                            autodiff_model,
                                                                            source_link_name,
                                                                            target_link_name,
                                                                            desired_pose,
                                                                            q0));
        // 2. Choose solver and options
        ifopt::IpoptSolver ipopt;
        ipopt.SetOption("mu_strategy", "adaptive");
        ipopt.SetOption("jacobian_approximation", "exact");
        ipopt.SetOption("max_iter", 1000);
        ipopt.SetOption("tol", 1e-3);
        ipopt.SetOption("print_level", 0);
        ipopt.SetOption("sb", "yes");
        // 3. Solve
        ipopt.Solve(nlp);
        Eigen::Matrix<Scalar, nq, 1> q = nlp.GetOptVariables()->GetValues();
        return q;
    }
}  // namespace RML

#endif
