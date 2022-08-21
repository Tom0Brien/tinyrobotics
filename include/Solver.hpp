#ifndef RML_SOLVERS_HPP
#define RML_SOLVERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Dynamics.hpp"
#include "Model.hpp"

namespace RML {

    /// @brief The types of integration methods
    namespace IntegrationMethod {
        struct EULER {};
        struct SYMPLECTIC_EULER {};
    };  // namespace IntegrationMethod

    // enum struct IntegrationMethod { EULER, SYMPLECTIC_EULER };

    /**
     * @brief Perform a euler integration step.
     * @param model The robot model.
     * @param qkm1 The previous joint configuration.
     * @param pkm1 The previous momentum.
     * @param dt The time step.
     * @return The next time steps joint configuration and momentum [qk; pk].
     */
    template <typename Scalar, int nq, int np, int ni>
    Eigen::Matrix<Scalar, nq + np, 1> integration_step(Model<Scalar>& model,
                                                       const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                       const Eigen::Matrix<Scalar, np, 1>& pkm1,
                                                       const Eigen::Matrix<Scalar, ni, 1>& ukm1,
                                                       const float& dt,
                                                       const IntegrationMethod::EULER&) {
        Eigen::Matrix<Scalar, nq, 1> qk;
        Eigen::Matrix<Scalar, np, 1> pk;
        Eigen::Matrix<Scalar, nq + np, 1> xk;

        // Compute the dynamics of the system
        hamiltonian_dynamics(model, qkm1, pkm1, ukm1);

        // Perform the q update: qk = qkm1 + dt*dqm1dt
        qk = qkm1 + dt * model.results.dx_dt.head(nq);

        // Perform the p update: pk = pkm1 + dt*dpm1dt
        pk = pkm1 + dt * model.results.dx_dt.tail(np);

        // Store the new state
        xk << qk, pk;
        return xk;
    }

    /**
     * @brief Perform a symplectic euler integration step.
     * @param model The robot model.
     * @param qkm1 The previous joint configuration.
     * @param pkm1 The previous momentum.
     * @param dt The time step.
     * @return The next time steps configuration and momentum [qk; pk].
     */
    template <typename Scalar, int nq, int np, int ni>
    Eigen::Matrix<Scalar, nq + np, 1> integration_step(Model<Scalar>& model,
                                                       const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                       const Eigen::Matrix<Scalar, np, 1>& pkm1,
                                                       const Eigen::Matrix<Scalar, ni, 1>& ukm1,
                                                       const float& dt,
                                                       const IntegrationMethod::SYMPLECTIC_EULER&) {
        Eigen::Matrix<Scalar, nq, 1> qk;
        Eigen::Matrix<Scalar, np, 1> pk;
        Eigen::Matrix<Scalar, nq + np, 1> xk;

        // Compute the mass matrix, which can be used for q update
        Eigen::Matrix<Scalar, np, nq> Mkm1 = mass_matrix(model, qkm1);

        // Perform the q update: qk = qkm1 + dt*dqm1dt
        qk = qkm1 + dt * (Mkm1.inverse() * pkm1);

        // Compute the dynamics of the system with qk and pkm1 for p update
        hamiltonian_dynamics(model, qk, pkm1, ukm1);

        // Perform the p update: pk = pkm1 + dt*dpm1dt
        pk = pkm1 + dt * model.results.dx_dt.tail(np);

        // Store the new state
        xk << qk, pk;
        return xk;
    }

    /**
     * @brief Solve ODE system of the form dx/dt = f(t,x)
     * @param model The robot model.
     * @param q0 The initial joint configuration of the robot.
     * @param p0 The initial momentum of the robot.
     * @param u0 The initial control input of the robot.
     * @param tspan The time span of the integration interval [t0 tf].
     * @param integration_method The integration method to use.
     * @return A vector of states of the robot integrated over the time span.
     */
    template <typename Scalar, int nq, int np, int ni, typename... Args>
    std::vector<Eigen::Matrix<Scalar, nq + np, 1>> solver(Model<Scalar>& model,
                                                          Eigen::Matrix<Scalar, nq, 1> qk,
                                                          Eigen::Matrix<Scalar, np, 1> pk,
                                                          Eigen::Matrix<Scalar, ni, 1> u,
                                                          const Eigen::Matrix<Scalar, 2, 1>& tspan,
                                                          const float& dt,
                                                          const Args&... params) {
        // Create a vector of states for the robot dynamics integrated over the time span.
        std::vector<Eigen::Matrix<Scalar, nq + np, 1>> x_history;

        // Ensure that the time span is valid
        if (tspan(0) >= tspan(1)) {
            throw std::invalid_argument("The time span is invalid.");
        }
        // Ensure that the time step is valid
        if (dt <= 0 || dt >= tspan(1) - tspan(0)) {
            throw std::invalid_argument("The time step is invalid.");
        }

        // For each time step in the time span, integrate the robot dynamics.
        const int n_steps = (tspan(1) - tspan(0)) / dt + 2;
        for (int k = 0; k < n_steps; k++) {
            // Compute the current time
            const float t = tspan(0) + k * dt;
            // TODO: Compute the control input
            // u = controller(model,qk,pk,t);
            // Perform the integration step
            x_history.push_back(integration_step(model, qk, pk, u, dt, params...));
            // Update the initial state for the next integration step
            qk = x_history.back().head(nq);
            pk = x_history.back().tail(np);
        }
        return x_history;
    }
}  // namespace RML


#endif  // RML_SOLVERS_HPP