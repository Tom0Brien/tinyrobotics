namespace tinyrobotics {

    template <typename Scalar, int nq>
    Scalar Model<Scalar, nq>::cost(const Eigen::Matrix<Scalar, nq, 1>& q,
                                   const std::string& target_link_name,
                                   const std::string& source_link_name,
                                   const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                                   const Eigen::Matrix<Scalar, nq, 1>& q0,
                                   Eigen::Matrix<Scalar, nq, 1>& gradient,
                                   const InverseKinematicsOptions<Scalar, nq>& options) {

        // Compute the current pose
        Eigen::Transform<Scalar, 3, Eigen::Isometry> current_pose =
            forwardKinematics(q, target_link_name, source_link_name);

        // Compute the pose error
        Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneousError(current_pose, desired_pose);

        // Compute the cost: q^T*W*q + (k(q) - x*)^TK*(k(q) - x*))), where k(q) is the current pose, x* is the desired
        // pose, and W and K are the weighting matrices
        Eigen::Matrix<Scalar, 1, 1> cost =
            0.5 * pose_error.transpose() * options.K * pose_error + 0.5 * (q - q0).transpose() * options.W * (q - q0);

        // Compute the gradient of the cost function
        Eigen::Matrix<Scalar, 6, nq> J = jacobian(q, target_link_name);
        gradient                       = J.transpose() * options.K * pose_error + options.W * (q - q0);

        return cost(0);
    }


    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematicsNLOPT(
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
            cost_value        = cost(q, target_link_name, source_link_name, desired_pose, q0, gradient, options);

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


    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematicsJacobian(
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
                forwardKinematics(q_current, target_link_name, source_link_name);

            // Compute the pose error vector
            Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneousError(current_pose, desired_pose);

            // Check if the error is within tolerance
            if (pose_error.norm() < options.tolerance) {
                break;
            }

            // Compute the Jacobian matrix
            Eigen::Matrix<Scalar, 6, nq> J = jacobian(q_current, target_link_name);

            // Compute the change in configuration
            Eigen::Matrix<Scalar, nq, 1> delta_q =
                J.completeOrthogonalDecomposition().pseudoInverse() * (-options.step_size * pose_error);

            // Update the current configuration
            q_current += delta_q;
        }

        // Return the final configuration
        return q_current;
    }


    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematicsLevenbergMarquardt(
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
                forwardKinematics(q_current, target_link_name, source_link_name);

            // Compute the pose error vector
            Eigen::Matrix<Scalar, 6, 1> pose_error = homogeneousError(current_pose, desired_pose);

            // Check if the error is within tolerance
            if (pose_error.norm() < options.tolerance) {
                break;
            }

            // Compute the Jacobian matrix
            Eigen::Matrix<Scalar, 6, nq> J = jacobian(q_current, target_link_name);

            // Compute the Hessian approximation and the gradient
            Eigen::Matrix<Scalar, nq, nq> H = J.transpose() * J;
            Eigen::Matrix<Scalar, nq, 1> g  = J.transpose() * pose_error;

            // Levenberg-Marquardt update
            Eigen::Matrix<Scalar, nq, 1> delta_q =
                (H + lambda * Eigen::Matrix<Scalar, nq, nq>(H.diagonal().asDiagonal())).ldlt().solve(-g);

            // Test the new configuration
            Eigen::Matrix<Scalar, nq, 1> q_new = q_current + delta_q;
            Eigen::Transform<Scalar, 3, Eigen::Isometry> new_pose =
                forwardKinematics(q_new, target_link_name, source_link_name);

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


    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematicsPSO(
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
                    forwardKinematics(particles[i], target_link_name, source_link_name);
                Scalar fitness_value = homogeneousError(current_pose, desired_pose).squaredNorm();

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

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematicsBFGS(
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
            Scalar cost_value = cost(q, target_link_name, source_link_name, desired_pose, q0, grad, options);

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
                cost_new = cost(q_new, target_link_name, source_link_name, desired_pose, q0, grad_new, options);
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

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseKinematics(
        const std::string& target_link_name,
        const std::string& source_link_name,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
        const Eigen::Matrix<Scalar, nq, 1> q0,
        const InverseKinematicsOptions<Scalar, nq>& options) {
        switch (options.method) {
            case InverseKinematicsMethod::NLOPT:
                return inverseKinematicsNLOPT(target_link_name, source_link_name, desired_pose, q0, options);
            case InverseKinematicsMethod::JACOBIAN:
                return inverseKinematicsJacobian(target_link_name, source_link_name, desired_pose, q0, options);
            case InverseKinematicsMethod::LEVENBERG_MARQUARDT:
                return inverseKinematicsLevenbergMarquardt(target_link_name,
                                                           source_link_name,
                                                           desired_pose,
                                                           q0,
                                                           options);
            case InverseKinematicsMethod::PARTICLE_SWARM:
                return inverseKinematicsPSO(target_link_name, source_link_name, desired_pose, q0, options);
            case InverseKinematicsMethod::BFGS:
                return inverseKinematicsBFGS(target_link_name, source_link_name, desired_pose, q0, options);
            default: throw std::runtime_error("Unknown inverse kinematics method");
        }
    }

}  // namespace tinyrobotics
