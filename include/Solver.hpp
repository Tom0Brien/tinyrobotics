#ifndef TR_SOLVERS_HPP
#define TR_SOLVERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "Dynamics.hpp"
#include "Model.hpp"

/** \file Solver.hpp
 * @brief Contains various functions for integrating the dynamics of a tinyrobotics model.
 */
namespace tr::solver {

    using namespace tr::math;
    using namespace tr::model;
    using namespace tr::dynamics;

    /// @brief The types of integration methods
    enum class IntegrationMethod { EULER, SYMPLECTIC_EULER, RK4 };  // namespace IntegrationMethod

    /// @brief Struct for solver params
    template <typename Scalar, int nq>
    struct SolverParams {
        /// @brief The time step
        Scalar dt = 0.01;

        /// @brief The number of iterations
        std::pair<Scalar, Scalar> tspan = std::pair<Scalar, Scalar>(0, 10);

        /// @brief The integration method
        IntegrationMethod integration_method;

        /// @brief The initial active constraints
        std::vector<std::string> active_constraints = {};

        /// @brief Function pointer to event detection function
        bool (*event_detection)(Model<Scalar, nq>&,
                                Eigen::Matrix<Scalar, nq, 1>&,
                                Eigen::Matrix<Scalar, nq, 1>&) = nullptr;

        /// @brief Bool to indicate if the solver should stop on event detection
        bool event_is_terminal = false;
    };

    /// @brief Struct for solver results
    template <typename Scalar, int nq>
    struct SolverResults {
        std::vector<Eigen::Matrix<Scalar, nq + nq, 1>> x_history;
        std::vector<Eigen::Matrix<Scalar, nq, 1>> u_history;
        std::vector<Scalar> time;
    };

    /**
     * @brief Perform a euler integration step.
     * @param model The robot model.
     * @param qkm1 The previous joint configuration.
     * @param pkm1 The previous momentum.
     * @param dt The time step.
     * @return The next time steps joint configuration and momentum [qk; pk].
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq + nq, 1> euler_step(Model<Scalar, nq>& model,
                                                 const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                 const Eigen::Matrix<Scalar, nq, 1>& pkm1,
                                                 const Eigen::Matrix<Scalar, nq, 1>& ukm1,
                                                 const float& dt,
                                                 const std::vector<std::string> active_constraints = {}) {
        // Compute the dynamics of the system
        forward_dynamics(model, qkm1, pkm1, ukm1, active_constraints);

        // Perform the q update: qk = qkm1 + dt*dqm1dt
        Eigen::Matrix<Scalar, nq, 1> qk = qkm1 + dt * model.data.dx_dt.head(nq);

        // Perform the p update: pk = pkm1 + dt*dpm1dt
        Eigen::Matrix<Scalar, nq, 1> pk = pkm1 + dt * model.data.dx_dt.tail(nq);

        // Store the new state
        Eigen::Matrix<Scalar, nq + nq, 1> xk;
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
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq + nq, 1> symplectic_euler_step(Model<Scalar, nq>& model,
                                                            const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                            const Eigen::Matrix<Scalar, nq, 1>& pkm1,
                                                            const Eigen::Matrix<Scalar, nq, 1>& ukm1,
                                                            const float& dt,
                                                            const std::vector<std::string> active_constraints = {}) {
        // Compute the mass matrix, which can be used for q update
        Eigen::Matrix<Scalar, nq, nq> Mkm1 = mass_matrix(model, qkm1);

        // Perform the q update: qk = qkm1 + dt*dqm1dt
        Eigen::Matrix<Scalar, nq, 1> qk = qkm1 + dt * (Mkm1.inverse() * pkm1);

        // Compute the dynamics of the system with qk and pkm1 for p update
        forward_dynamics(model, qk, pkm1, ukm1, active_constraints);

        // Perform the p update: pk = pkm1 + dt*dpm1dt
        Eigen::Matrix<Scalar, nq, 1> pk = pkm1 + dt * model.data.dx_dt.tail(nq);

        // Store the new state
        Eigen::Matrix<Scalar, nq + nq, 1> xk;
        xk << qk, pk;
        return xk;
    }

    /**
     * @brief Perform a runge-kutta 4th order integration step.
     * @param model The robot model.
     * @param qkm1 The previous joint configuration.
     * @param pkm1 The previous momentum.
     * @param dt The time step.
     * @return The next time steps configuration and momentum [qk; pk].
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq + nq, 1> RK4(Model<Scalar, nq>& model,
                                          const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                          const Eigen::Matrix<Scalar, nq, 1>& pkm1,
                                          const Eigen::Matrix<Scalar, nq, 1>& ukm1,
                                          const float& dt,
                                          const std::vector<std::string> active_constraints = {}) {
        Eigen::Matrix<Scalar, nq + nq, 1> k1, k2, k3, k4, xk;
        // Compute the mass matrix, which can be used for q update
        k1 = forward_dynamics(model, qkm1, pkm1, ukm1, active_constraints);
        k2 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + 0.5 * dt * k1.head(nq)),
                              Eigen::Matrix<Scalar, nq, 1>(pkm1 + 0.5 * dt * k1.tail(nq)),
                              ukm1,
                              active_constraints);
        k3 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + 0.5 * dt * k2.head(nq)),
                              Eigen::Matrix<Scalar, nq, 1>(pkm1 + 0.5 * dt * k2.tail(nq)),
                              ukm1,
                              active_constraints);
        k4 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + dt * k3.head(nq)),
                              Eigen::Matrix<Scalar, nq, 1>(pkm1 + dt * k3.tail(nq)),
                              ukm1,
                              active_constraints);
        // Store the new state
        xk << qkm1 + dt / 6 * (k1.head(nq) + 2 * k2.head(nq) + 2 * k3.head(nq) + k4.head(nq)),
            pkm1 + dt / 6 * (k1.tail(nq) + 2 * k2.tail(nq) + 2 * k3.tail(nq) + k4.tail(nq));
        return xk;
    }

    /**
     * @brief Perform an integration step.
     * @param model The robot model.
     * @param qkm1 The previous joint configuration.
     * @param pkm1 The previous momentum.
     * @param dt The time step.
     * @return The next time steps configuration and momentum [qk; pk].
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq + nq, 1> integrate(Model<Scalar, nq>& model,
                                                const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                const Eigen::Matrix<Scalar, nq, 1>& pkm1,
                                                const Eigen::Matrix<Scalar, nq, 1>& ukm1,
                                                const float& dt,
                                                const IntegrationMethod& integration_type,
                                                const std::vector<std::string> active_constraints) {
        // Perform the integration step based on the integration method
        switch (integration_type) {
            case IntegrationMethod::EULER: return euler_step(model, qkm1, pkm1, ukm1, dt, active_constraints);
            case IntegrationMethod::SYMPLECTIC_EULER:
                return symplectic_euler_step(model, qkm1, pkm1, ukm1, dt, active_constraints);
            case IntegrationMethod::RK4: return RK4(model, qkm1, pkm1, ukm1, dt, active_constraints);
            default: throw std::runtime_error("Unknown integration method.");
        }
    }


    /**
     * @brief Save the history of the simulation output as a json file for urdf-visualizer.
     * @param model The robot model.
     * @param x_history The simulation history.
     */
    template <typename Scalar, int nq>
    void save_history(Model<Scalar, nq>& model, std::vector<Eigen::Matrix<Scalar, nq + nq, 1>> x_history) {
        nlohmann::json json_q_history;
        // Loop through the simulation history and save the q values with joint names
        for (int i = 0; i < x_history.size(); i++) {
            for (auto joint : model.joints) {
                if (joint.q_idx != -1) {
                    json_q_history[std::to_string(i)][joint.name] = {x_history[i](joint.q_idx)};
                }
            }
        }
        // Write the json result to a file
        std::ofstream o(model.name + "solver_history.json");
        o << std::setw(4) << json_q_history << std::endl;
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
    template <typename Scalar, int nq, typename... IntMethod>
    SolverResults<Scalar, nq> solver(Model<Scalar, nq>& model,
                                     Eigen::Matrix<Scalar, nq, 1> qk,
                                     Eigen::Matrix<Scalar, nq, 1> pk,
                                     Eigen::Matrix<Scalar, nq, 1> u,
                                     SolverParams<Scalar, nq>& params) {
        // Create a struct for solver results
        SolverResults<Scalar, nq> results;

        // Ensure that the time span is valid
        if (params.tspan.first >= params.tspan.second) {
            throw std::invalid_argument("The time span is invalid.");
        }
        // Ensure that the time step is valid
        if (params.dt <= 0 || params.dt >= params.tspan.second - params.tspan.first) {
            throw std::invalid_argument("The time step is invalid.");
        }

        // For each time step in the time span, integrate the robot dynamics.
        const int n_steps = (params.tspan.second - params.tspan.first) / params.dt + 2;
        for (int k = 0; k < n_steps; k++) {
            // Compute the current time
            const Scalar t = params.tspan.first + k * params.dt;
            results.time.push_back(t);
            // TODO: Compute the control input
            // u = controller(model,qk,pk,t);
            results.u_history.push_back(u);
            // Perform the integration step
            results.x_history.push_back(
                integrate(model, qk, pk, u, params.dt, params.integration_method, params.active_constraints));
            // Update the initial state for the next integration step
            qk = results.x_history.back().head(nq);
            pk = results.x_history.back().tail(nq);
            // Check if event has occurred
            if (params.event_detection != nullptr) {
                if (params.event_detection(model, qk, pk)) {
                    std::cout << "Event occurred at time: " << t << std::endl;
                    if (params.event_is_terminal) {
                        break;
                    }
                }
            }
        }
        return results;
    }

}  // namespace tr::solver


#endif  // TR_SOLVERS_HPP