#ifndef RML_SOLVERS_HPP
#define RML_SOLVERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "Dynamics.hpp"
#include "Model.hpp"
#include "matplotlibcpp.h"

namespace RML {

    /// @brief The types of integration methods
    namespace IntegrationMethod {
        struct EULER {};
        struct SYMPLECTIC_EULER {};
        struct RK4 {};
    };  // namespace IntegrationMethod

    // @brief Struct for solver results
    template <typename Scalar, int nq, int np, int ni>
    struct SolverResults {
        std::vector<Eigen::Matrix<Scalar, nq + np, 1>> x_history;
        std::vector<Eigen::Matrix<Scalar, ni, 1>> u_history;
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
        forward_dynamics(model, qkm1, pkm1, ukm1);

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
        forward_dynamics(model, qk, pkm1, ukm1);

        // Perform the p update: pk = pkm1 + dt*dpm1dt
        pk = pkm1 + dt * model.results.dx_dt.tail(np);

        // Store the new state
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
    template <typename Scalar, int nq, int np, int ni>
    Eigen::Matrix<Scalar, nq + np, 1> integration_step(Model<Scalar>& model,
                                                       const Eigen::Matrix<Scalar, nq, 1>& qkm1,
                                                       const Eigen::Matrix<Scalar, np, 1>& pkm1,
                                                       const Eigen::Matrix<Scalar, ni, 1>& ukm1,
                                                       const float& dt,
                                                       const IntegrationMethod::RK4&) {
        Eigen::Matrix<Scalar, nq + np, 1> k1, k2, k3, k4, xk;

        // Compute the mass matrix, which can be used for q update
        k1 = forward_dynamics(model, qkm1, pkm1, ukm1);
        k2 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + 0.5 * dt * k1.head(nq)),
                              Eigen::Matrix<Scalar, nq, 1>(pkm1 + 0.5 * dt * k1.tail(np)),
                              ukm1);
        k3 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + 0.5 * dt * k2.head(nq)),
                              Eigen::Matrix<Scalar, np, 1>(pkm1 + 0.5 * dt * k2.tail(np)),
                              ukm1);
        k4 = forward_dynamics(model,
                              Eigen::Matrix<Scalar, nq, 1>(qkm1 + dt * k3.head(nq)),
                              Eigen::Matrix<Scalar, np, 1>(pkm1 + dt * k3.tail(np)),
                              ukm1);
        // Store the new state
        xk << qkm1 + dt / 6 * (k1.head(nq) + 2 * k2.head(nq) + 2 * k3.head(nq) + k4.head(nq)),
            pkm1 + dt / 6 * (k1.tail(np) + 2 * k2.tail(np) + 2 * k3.tail(np) + k4.tail(np));
        return xk;
    }

    /**
     * @brief Save the history of the simulation output as a json file for urdf-visualizer.
     * @param model The robot model.
     * @param x_history The simulation history.
     */
    template <typename Scalar, int nx>
    void save_history(Model<Scalar>& model, std::vector<Eigen::Matrix<Scalar, nx, 1>> x_history) {
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
        std::ofstream o("test_joint_set.json");
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
    template <typename Scalar, int nq, int np, int ni, typename... Args>
    SolverResults<Scalar, nq, np, ni> solver(Model<Scalar>& model,
                                             Eigen::Matrix<Scalar, nq, 1> qk,
                                             Eigen::Matrix<Scalar, np, 1> pk,
                                             Eigen::Matrix<Scalar, ni, 1> u,
                                             const Eigen::Matrix<Scalar, 2, 1>& tspan,
                                             const float& dt,
                                             const Args&... params) {
        // Create a struct for solver results
        SolverResults<Scalar, nq, np, ni> results;

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
            results.time.push_back(t);
            // TODO: Compute the control input
            // u = controller(model,qk,pk,t);
            results.u_history.push_back(u);
            // Perform the integration step
            results.x_history.push_back(integration_step(model, qk, pk, u, dt, params...));
            // Update the initial state for the next integration step
            qk = results.x_history.back().head(nq);
            pk = results.x_history.back().tail(np);
        }
        return results;
    }


    /**
     * @brief Plot the results of a simulation.
     * @param results The results of the simulation.
     */
    template <typename Scalar, int nq, int np, int ni>
    void plot_results(const SolverResults<Scalar, nq, np, ni>& results) {
        // Plot the generalised coordinates against time
        for (int i = 0; i < nq; i++) {
            std::vector<Scalar> x;
            // Add the results of the states to the vector x
            for (int j = 0; j < results.x_history.size(); j++) {
                x.push_back(results.x_history[j](i));
            }
            // Plot the results
            matplotlibcpp::suptitle("Simulation Results");
            matplotlibcpp::subplot2grid(nq, 3, i, 0);
            matplotlibcpp::plot(results.time, x);
            matplotlibcpp::title("Generalised Coordinate q" + std::to_string(i));
            matplotlibcpp::xlabel("Time [s]");
            matplotlibcpp::ylabel("q" + std::to_string(i));
            matplotlibcpp::grid(true);
            matplotlibcpp::show(false);
        }

        // Plot the generalised momentum against time
        for (int i = 0; i < np; i++) {
            std::vector<Scalar> x;
            // Add the results of the states to the vector x
            for (int j = 0; j < results.x_history.size(); j++) {
                x.push_back(results.x_history[j](nq + i));
            }
            // Plot the results
            matplotlibcpp::subplot2grid(nq, 3, i, 1);
            matplotlibcpp::plot(results.time, x);
            matplotlibcpp::title("Generalised Momentum q" + std::to_string(i));
            matplotlibcpp::xlabel("Time [s]");
            matplotlibcpp::ylabel("p" + std::to_string(i));
            matplotlibcpp::grid(true);
            matplotlibcpp::show(false);
        }

        // Plot the inputs against time
        for (int i = 0; i < ni; i++) {
            std::vector<Scalar> u;
            // Add the results of the states to the vector x2
            for (int j = 0; j < results.u_history.size(); j++) {
                u.push_back(results.u_history[j](i));
            }
            // Plot the results
            matplotlibcpp::subplot2grid(nq, 3, i, 2);
            matplotlibcpp::plot(results.time, u);
            matplotlibcpp::title("Input u" + std::to_string(i));
            matplotlibcpp::xlabel("Time [s]");
            matplotlibcpp::ylabel("u" + std::to_string(i));
            matplotlibcpp::grid(true);
            matplotlibcpp::show(false);
        }
        matplotlibcpp::show(true);
    }

}  // namespace RML


#endif  // RML_SOLVERS_HPP