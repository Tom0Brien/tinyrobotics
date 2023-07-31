namespace tinyrobotics {

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, nq> Model<Scalar, nq>::massMatrix(const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Reset mass matrix to zero
        mass_matrix.setZero();

        // First pass:
        for (int i = 0; i < nq; i++) {
            // Compute the spatial transform from the parent to the current body
            Xup[i] = homogeneous_to_spatial(links[q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            IC[i]  = links[q_map[i]].I;
        }

        // Second pass:
        for (int i = nq - 1; i >= 0; i--) {
            if (parent[i] != -1) {
                IC[parent[i]] = IC[parent[i]] + Xup[i].transpose() * IC[i] * Xup[i];
            }
        }

        // Third pass:
        for (int i = 0; i < nq; i++) {
            fh                = IC[i] * links[q_map[i]].joint.S;
            mass_matrix(i, i) = links[q_map[i]].joint.S.transpose() * fh;
            int j             = i;
            while (parent[j] > -1) {
                fh                = Xup[j].transpose() * fh;
                j                 = parent[j];
                mass_matrix(i, j) = links[q_map[j]].joint.S.transpose() * fh;
                mass_matrix(j, i) = mass_matrix(i, j);
            }
        }

        return mass_matrix;
    }

    template <typename Scalar, int nq>
    Scalar Model<Scalar, nq>::kineticEnergy(const Eigen::Matrix<Scalar, nq, 1>& q,
                                            const Eigen::Matrix<Scalar, nq, 1>& dq) {
        // Compute the mass matrix
        massMatrix(q);

        // Compute the kinetic energy
        kinetic_energy = Scalar(0.5) * dq.transpose() * mass_matrix * dq;
        return kinetic_energy;
    }

    template <typename Scalar, int nq>
    Scalar Model<Scalar, nq>::potentialEnergy(const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Reset the potential energy
        potential_energy = Scalar(0.0);

        // Compute the forward kinematics to centre of mass of all links
        forwardKinematicsCOM(q);

        // Compute the potential energy
        for (auto const link : links) {
            potential_energy += -link.mass * gravity.transpose() * forward_kinematics_com[link.idx].translation();
        }
        return potential_energy;
    }

    template <typename Scalar, int nq>
    Scalar Model<Scalar, nq>::totalEnergy(const Eigen::Matrix<Scalar, nq, 1>& q,
                                          const Eigen::Matrix<Scalar, nq, 1>& dq) {
        // Compute and return the total energy (kinetic + potential)
        total_energy = kineticEnergy(q, dq) + potentialEnergy(q);
        return total_energy;
    }

    template <typename Scalar, int nq>
    std::vector<Eigen::Matrix<Scalar, 6, 1>> Model<Scalar, nq>::applyExternalForces(
        const std::vector<Eigen::Matrix<Scalar, 6, 6>>& Xup,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_in,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_external) {
        std::vector<Eigen::Matrix<Scalar, 6, 1>> f_out(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xa(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        f_out = f_in;
        for (int i = 0; i < nq; i++) {
            const auto link = links[q_map[i]];
            if (parent[i] == -1) {
                Xa[i] = Xup[i];
            }
            else {
                Xa[i] = Xup[i] * Xa[parent[i]];
            }
            f_out[i] += Xa[i].transpose().inverse() * f_external[i];
        }
        return f_out;
    }

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::forwardDynamics(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const Eigen::Matrix<Scalar, nq, 1>& qd,
        const Eigen::Matrix<Scalar, nq, 1>& tau,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_external) {

        // First pass: compute the spatial acceleration of each body
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            vJ = links[q_map[i]].joint.S * qd(i);
            // Compute the spatial transform from the parent to the current body
            Xup[i] = homogeneous_to_spatial(links[q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the parent link is the base link
            if (parent[i] == -1) {
                v[i] = vJ;
                c[i] = Eigen::Matrix<Scalar, 6, 1>::Zero();
            }
            else {
                v[i] = Xup[i] * v[parent[i]] + vJ;
                c[i] = cross_spatial(v[i]) * vJ;
            }
            IA[i] = links[q_map[i]].I;
            pA[i] = cross_motion(v[i]) * IA[i] * v[i];
        }

        // Apply external forces if non-zero
        if (!f_external.empty()) {
            pA = applyExternalForces(Xup, pA, f_external);
        }

        // Second pass: compute the spatial force acting on each body
        for (int i = nq - 1; i >= 0; i--) {
            U[i] = IA[i] * links[q_map[i]].joint.S;
            d[i] = links[q_map[i]].joint.S.transpose() * U[i];
            u[i] = Scalar(tau(i) - links[q_map[i]].joint.S.transpose() * pA[i]);
            if (parent[i] != -1) {
                Eigen::Matrix<Scalar, 6, 6> Ia = IA[i] - (U[i] / d[i]) * U[i].transpose();
                Eigen::Matrix<Scalar, 6, 1> pa = pA[i] + Ia * c[i] + U[i] * (u[i] / d[i]);
                IA[parent[i]] += Xup[i].transpose() * Ia * Xup[i];
                pA[parent[i]] += Xup[i].transpose() * pa;
            }
        }

        // Third pass: compute the joint accelerations
        for (int i = 0; i < nq; i++) {
            if (parent[i] == -1) {
                a[i] = Xup[i] * -spatial_gravity + c[i];
            }
            else {
                a[i] = Xup[i] * a[parent[i]] + c[i];
            }
            ddq(i) = (u[i] - U[i].transpose() * a[i]) / d[i];
            a[i]   = a[i] + links[q_map[i]].joint.S * ddq(i);
        }
        return ddq;
    }

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::forwardDynamicsCRB(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const Eigen::Matrix<Scalar, nq, 1>& qd,
        const Eigen::Matrix<Scalar, nq, 1>& tau,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_external) {
        // First pass:
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            vJ = links[q_map[i]].joint.S * qd(i);
            // Compute the spatial transform from the parent to the current body
            Xup[i] = homogeneous_to_spatial(links[q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the parent link is the base link
            if (parent[i] == -1) {
                v[i]   = vJ;
                avp[i] = Xup[i] * -spatial_gravity;
            }
            else {
                v[i]   = Xup[i] * v[parent[i]] + vJ;
                avp[i] = Xup[i] * avp[parent[i]] + cross_spatial(v[i]) * vJ;
            }
            fvp[i] = links[q_map[i]].I * avp[i] + cross_motion(v[i]) * links[q_map[i]].I * v[i];
            IC[i]  = links[q_map[i]].I;
        }

        // Apply external forces if non-zero
        if (!f_external.empty()) {
            fvp = applyExternalForces(Xup, pA, f_external);
        }

        // Second pass:
        for (int i = nq - 1; i >= 0; i--) {
            C(i, 0) = links[q_map[i]].joint.S.transpose() * fvp[i];
            if (parent[i] != -1) {
                fvp[parent[i]] = fvp[parent[i]] + Xup[i].transpose() * fvp[i];
            }
        }

        // Third pass:
        for (int i = nq - 1; i >= 0; i--) {
            if (parent[i] != -1) {
                IC[parent[i]] = IC[parent[i]] + Xup[i].transpose() * IC[i] * Xup[i];
            }
        }

        for (int i = 0; i < nq; i++) {
            fh                = IC[i] * links[q_map[i]].joint.S;
            mass_matrix(i, i) = links[q_map[i]].joint.S.transpose() * fh;
            int j             = i;
            while (parent[j] != -1) {
                fh                = Xup[j].transpose() * fh;
                j                 = parent[j];
                mass_matrix(i, j) = links[q_map[j]].joint.S.transpose() * fh;
                mass_matrix(j, i) = mass_matrix(i, j);
            }
        }

        // Compute qdd with inverse mass using LDLT
        Eigen::Matrix<Scalar, nq, 1> qdd = mass_matrix.ldlt().solve(tau - C);

        return qdd;
    }

    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, nq, 1> Model<Scalar, nq>::inverseDynamics(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const Eigen::Matrix<Scalar, nq, 1>& qd,
        const Eigen::Matrix<Scalar, nq, 1>& qdd,
        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_external) {
        // First pass:
        for (int i = 0; i < nq; i++) {
            // Compute the joint transform and motion subspace matrices
            links[q_map[i]].joint.S = links[q_map[i]].joint.S;
            vJ                      = links[q_map[i]].joint.S * qd(i);
            // Compute the spatial transform from the parent to the current body
            Xup[i] = homogeneous_to_spatial(links[q_map[i]].joint.get_parent_to_child_transform(q(i)).inverse());
            // Check if the parent link is the base link
            if (parent[i] == -1) {
                v[i] = vJ;
                a[i] = Xup[i] * -spatial_gravity + links[q_map[i]].joint.S * qdd(i);
            }
            else {
                v[i] = Xup[i] * v[parent[i]] + vJ;
                a[i] = Xup[i] * a[parent[i]] + links[q_map[i]].joint.S * qdd(i) + cross_spatial(v[i]) * vJ;
            }
            fvp[i] = links[q_map[i]].I * a[i] + cross_motion(v[i]) * links[q_map[i]].I * v[i];
        }

        // Apply external forces if non-zero
        if (!f_external.empty()) {
            fvp = applyExternalForces(Xup, pA, f_external);
        }

        // Second pass:
        for (int i = nq - 1; i >= 0; i--) {
            tau(i, 0) = links[q_map[i]].joint.S.transpose() * fvp[i];
            if (parent[i] != -1) {
                fvp[parent[i]] = fvp[parent[i]] + Xup[i].transpose() * fvp[i];
            }
        }
        return tau;
    }

}  // namespace tinyrobotics
