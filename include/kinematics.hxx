namespace tinyrobotics {
    /**
     * @brief Retrieves the index of the target link in the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Index of the target link in the model.
     * @throws std::invalid_argument if the TargetLink type is not int or std::string.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink>
    int Model<Scalar, nq>::getLinkIndex(const TargetLink& target_link) {
        if constexpr (std::is_integral<TargetLink>::value) {
            return static_cast<int>(target_link);
        }
        else if constexpr (std::is_same<TargetLink, std::string>::value) {
            return getLink(target_link).idx;
        }
        else {
            throw std::invalid_argument("Invalid target_link type. Must be int or std::string.");
        }
    }

    /**
     * @brief Computes the transform between target and the source link. The transform converts points in target
     * frame to the source link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @return Homogeneous transform between the target and the source link.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink, typename SourceLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> Model<Scalar, nq>::forwardKinematics(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const TargetLink& target_link,
        const SourceLink& source_link) {
        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbs = forwardKinematics(q, getLinkIndex(source_link));
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsb = Hbs.inverse();
        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbt = forwardKinematics(q, getLinkIndex(target_link));
        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the transform to all the links in the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Stores the transform to all the links in forward_kinematics.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> Model<Scalar, nq>::forwardKinematics(
        const Eigen::Matrix<Scalar, nq, 1>& q) {
        forward_kinematics.resize(links.size(), Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        for (auto const link : links) {
            forward_kinematics[link.idx] = link.joint.parent_transform;
            if (link.joint.idx != -1) {
                forward_kinematics[link.idx] =
                    forward_kinematics[link.idx] * link.joint.get_joint_transform(q[link.joint.idx]);
            }
            if (link.parent != -1) {
                forward_kinematics[link.idx] = forward_kinematics[link.parent] * forward_kinematics[link.idx];
            }
        }
        return forward_kinematics;
    }

    /**
     * @brief Computes the transform between target and the base link. The transform converts points in target
     * frame to the base link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Homogeneous transform between the target and the base link.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> Model<Scalar, nq>::forwardKinematics(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const TargetLink& target_link) {
        // Get the target link
        Link<Scalar> current_link = links[getLinkIndex(target_link)];

        // Build kinematic tree from target_link {t} to base {b}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htb = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        while (current_link.idx != base_link_idx) {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> H = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
            if (current_link.joint.idx != -1) {
                // Transform the joint frame by joint value (Hpt)
                H = current_link.joint.get_joint_transform(q[current_link.joint.idx]);
            }
            Htb = Htb * H.inverse();
            // Apply inverse joint transform as we are going back up tree
            Htb = Htb * current_link.joint.parent_transform.inverse();
            // Move up the tree to parent link
            current_link = links[current_link.parent];
        }
        return Htb.inverse();
    }

    /**
     * @brief Computes the transform between centre of mass of each link and the source link.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Vector of homogeneous transforms between the centre of mass of each link and the source link.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> Model<Scalar, nq>::forwardKinematicsCOM(
        const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute forward kinematics for all the links
        forwardKinematics(q);
        // Compute transform to centre of mass of all the forward kinematics results
        forward_kinematics_com.resize(links.size(), Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        for (auto const link : links) {
            forward_kinematics_com[link.idx] = forward_kinematics[link.idx] * links[link.idx].centreOfMass;
        }
        return forward_kinematics_com;
    }

    /**
     * @brief Computes the transform between source link and target centre of mass. The transform converts points in the
     * target links centre of mass frame into the source link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @return Homogeneous transform from source link to target link centre of mass.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink, typename SourceLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> Model<Scalar, nq>::forwardKinematicsCOM(
        const Eigen::Matrix<Scalar, nq, 1>& q,
        const TargetLink& target_link,
        const SourceLink& source_link) {
        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = forwardKinematics(q, target_link, source_link);
        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = Hst * links[getLinkIndex(target_link)].centreOfMass;
        return Hsc;
    }

    /**
     * @brief Computes the geometric jacobian of the target link from the base link, in the base link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return The geometric jacobian of the target link from the base link, in the base link frame.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink>
    Eigen::Matrix<Scalar, 6, nq> Model<Scalar, nq>::jacobian(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                             const TargetLink& target_link) {
        // Compute the transform between base {b} and all the links {i} in the kinematic chain
        forwardKinematics(q);

        // Get the target link from the model
        const int targetLinkIndex = getLinkIndex(target_link);

        // Get the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTBb = forward_kinematics[targetLinkIndex].translation();

        // Initialize variables
        J.setZero();
        Eigen::Matrix<Scalar, 3, 1> zIBb;
        Eigen::Matrix<Scalar, 3, 1> rIBb;
        Link<Scalar> current_link = links[targetLinkIndex];
        while (current_link.idx != base_link_idx) {
            if (current_link.joint.idx != -1) {
                // Axis of the current joint rotated into the base frame {b}
                zIBb = forward_kinematics[current_link.idx].linear() * current_link.joint.axis;
                // Compute the displacement of the current link {i} from the base link frame {b}
                rIBb = forward_kinematics[current_link.idx].translation();
                if (current_link.joint.type == JointType::PRISMATIC) {
                    J.block(0, current_link.joint.idx, 3, 1) = zIBb;
                    J.block(3, current_link.joint.idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_link.joint.type == JointType::REVOLUTE) {
                    J.block(0, current_link.joint.idx, 3, 1) = (zIBb).cross(rTBb - rIBb);
                    J.block(3, current_link.joint.idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = links[current_link.parent];
        }
        return J;
    }

    /**
     * @brief Computes the geometric jacobian of the target links from the base link, in the base link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return The geometric jacobian of the target link from the base link in the base link frame.
     */
    template <typename Scalar, int nq>
    template <typename TargetLink>
    Eigen::Matrix<Scalar, 6, nq> Model<Scalar, nq>::jacobianCOM(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                const TargetLink& target_link) {
        // Get the target link from the model
        Link<Scalar> current_link = links[getLinkIndex(target_link)];

        // Get the base link from the model
        auto base_link = links[base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTcBb = forwardKinematicsCOM(q, target_link, base_link.name).translation();

        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();
        while (current_link.idx != base_link_idx) {
            auto current_joint = current_link.joint;
            if (current_joint.idx != -1) {
                // Compute the transform between base {b} and the current link {i}
                auto Hbi = forwardKinematics(q, current_link.name, base_link.name);
                // Axis of the current joint rotated into the base frame {b}
                auto zIBb = Hbi.linear() * current_joint.axis;
                // Compute the displacement of the current link {i} from the base link frame {b}
                auto rIBb = Hbi.translation();
                if (current_joint.type == JointType::PRISMATIC) {
                    J.block(0, current_joint.idx, 3, 1) = zIBb;
                    J.block(3, current_joint.idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_joint.type == JointType::REVOLUTE) {
                    J.block(0, current_joint.idx, 3, 1) = (zIBb).cross(rTcBb - rIBb);
                    J.block(3, current_joint.idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = links[current_link.parent];
        }
        return J;
    }

    /**
     * @brief Computes the centre of mass expressed in source link frame.
     * @param model tinyrobotics model.
     * @param q The configuration vector of the robot model.
     * @param source_link Source link, which can be an integer (index) or a string (name), from which the centre of mass
     * is expressed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @return The centre of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq>
    template <typename SourceLink>
    Eigen::Matrix<Scalar, 3, 1> Model<Scalar, nq>::centreOfMass(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                const SourceLink& source_link) {
        // Initialize the centre of mass as zero
        Eigen::Matrix<Scalar, 3, 1> rCSs = Eigen::Matrix<Scalar, 3, 1>::Zero();
        for (auto link : links) {
            // Compute the transform from the source link to the CoM of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = forwardKinematicsCOM(q, link.name, source_link);
            // Compute the centre of mass of the link
            rCSs = rCSs + Hsc.translation() * link.mass;
        }
        // Compute the centre of mass from the source link
        return rCSs / mass;
    }
}  // namespace tinyrobotics