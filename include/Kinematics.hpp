#ifndef TR_FORWARDKINEMATICS_HPP
#define TR_FORWARDKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Math.hpp"
#include "Model.hpp"

/** \file Kinematics.hpp
 * @brief Contains functions for computing various kinematic quantities of a tinyrobotics model.
 */
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
    template <typename Scalar, int nq, typename TargetLink>
    const int get_target_link_idx(const Model<Scalar, nq>& model, const TargetLink& target_link) {
        if constexpr (std::is_integral<TargetLink>::value) {
            return static_cast<int>(target_link);
        }
        else if constexpr (std::is_same<TargetLink, std::string>::value) {
            return model.get_link(target_link).idx;
        }
        else {
            throw std::invalid_argument("Invalid target_link type. Must be int or std::string.");
        }
    }

    /**
     * @brief Computes the transform to all the links in the tinyrobotics model.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Stores the transform to all the links in model.data.forward_kinematics.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics(
        Model<Scalar, nq>& model,
        const Eigen::Matrix<Scalar, nq, 1>& q) {
        model.data.forward_kinematics.resize(model.n_links, Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        for (auto const link : model.links) {
            model.data.forward_kinematics[link.idx] = link.joint.parent_transform;
            if (link.joint.idx != -1) {
                model.data.forward_kinematics[link.idx] =
                    model.data.forward_kinematics[link.idx] * link.joint.get_joint_transform(q[link.joint.idx]);
            }
            if (link.parent != -1) {
                model.data.forward_kinematics[link.idx] =
                    model.data.forward_kinematics[link.parent] * model.data.forward_kinematics[link.idx];
            }
        }
        return model.data.forward_kinematics;
    }

    /**
     * @brief Computes the transform to the target from the base link.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @param inverse_transform OPTIONAL: Set true if you want the transform from the target to the source.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Homogeneous transform between the base and the target link.
     */
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const TargetLink& target_link,
                                                                    const bool inverse_transform = false) {
        // Get the target link
        const int target_link_idx = get_target_link_idx(model, target_link);
        Link<Scalar> current_link = model.links[target_link_idx];

        // Build kinematic tree from target_link {t} to base {b}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htb = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        while (current_link.idx != model.base_link_idx) {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> H = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
            if (current_link.joint.idx != -1) {
                // Transform the joint frame by joint value (Hpt)
                H = current_link.joint.get_joint_transform(q[current_link.joint.idx]);
            }
            Htb = Htb * H.inverse();
            // Apply inverse joint transform as we are going back up tree
            Htb = Htb * current_link.joint.parent_transform.inverse();
            // Move up the tree to parent link
            current_link = model.links[current_link.parent];
        }

        // If inverse transform is requested, return Hbt, which is transform from target {t} to base {b}
        if (inverse_transform) {
            return Htb;
        }
        // Return transform Hbt, which is transform from base {b} to target {t}
        return Htb.inverse();
    }

    /**
     * @brief Computes the homogeneous transform from a source to target link.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Homogeneous transform from source {s} to target {t} link.
     */
    template <typename Scalar, int nq, typename SourceLink, typename TargetLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const SourceLink& source_link,
                                                                    const TargetLink& target_link) {
        const int source_link_idx = get_target_link_idx(model, source_link);
        const int target_link_idx = get_target_link_idx(model, target_link);

        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsb = forward_kinematics(model, q, source_link_idx, true);
        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbt = forward_kinematics(model, q, target_link_idx);
        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the transform between source link {s} and target {t} centre of mass (CoM) {c}.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Homogeneous transform from source link {s} to target link centre of mass {c}.
     */
    template <typename Scalar, int nq, typename SourceLink, typename TargetLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(const Model<Scalar, nq>& model,
                                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                        const SourceLink& source_link,
                                                                        const TargetLink& target_link) {
        const int source_link_idx = get_target_link_idx(model, source_link);
        const int target_link_idx = get_target_link_idx(model, target_link);

        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = Hst * model.links[target_link_idx].centre_of_mass;
        // Return transform from source {s} to CoM {c}
        return Hsc;
    }

    /**
     * @brief Computes the translation of the target link from the source link in the source link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Translation of the target link from the source link in the source link frame.
     */
    template <typename Scalar, int nq, typename SourceLink, typename TargetLink>
    Eigen::Matrix<Scalar, 3, 1> translation(const Model<Scalar, nq>& model,
                                            const Eigen::Matrix<Scalar, nq, 1>& q,
                                            const SourceLink& source_link,
                                            const TargetLink& target_link) {
        const int source_link_idx = get_target_link_idx(model, source_link);
        const int target_link_idx = get_target_link_idx(model, target_link);

        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the rotation matrix between the target link and the source link in the source link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return Rotation matrix between the target link and the source link in the source link frame.
     */
    template <typename Scalar, int nq, typename SourceLink, typename TargetLink>
    Eigen::Matrix<Scalar, 3, 3> rotation(const Model<Scalar, nq>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         const SourceLink& source_link,
                                         const TargetLink& target_link) {
        int source_link_idx = get_target_link_idx(model, source_link);
        int target_link_idx = get_target_link_idx(model, target_link);

        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        return Hst.linear();
    }

    /**
     * @brief Computes the geometric jacobian of the target link from the base link in the base link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return The geometric jacobian of the target link from the base link in the base link frame.
     */
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian(const Model<Scalar, nq>& model,
                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                    const TargetLink& target_link) {
        // Get the target link from the model
        const int target_link_idx = get_target_link_idx(model, target_link);
        Link<Scalar> current_link = model.links[target_link_idx];

        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTBb = translation(model, q, base_link.idx, current_link.idx);

        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();
        while (current_link.idx != model.base_link_idx) {
            auto current_joint = current_link.joint;
            if (current_joint.idx != -1) {
                // Compute the transform between base {b} and the current link {i}
                auto Hbi = forward_kinematics(model, q, base_link.idx, current_link.idx);
                // Axis of the current joint rotated into the base frame {b}
                auto zIBb = Hbi.linear() * current_joint.axis;
                // Compute the displacement of the current link {i} from the base link frame {b}
                auto rIBb = Hbi.translation();
                if (current_joint.type == JointType::PRISMATIC) {
                    J.block(0, current_joint.idx, 3, 1) = zIBb;
                    J.block(3, current_joint.idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_joint.type == JointType::REVOLUTE) {
                    J.block(0, current_joint.idx, 3, 1) = (zIBb).cross(rTBb - rIBb);
                    J.block(3, current_joint.idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = model.links[current_link.parent];
        }
        return J;
    }

    /**
     * @brief Computes the geometric jacobian relative to the base for the specified target link's center of mass.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @return The geometric jacobian of the target link from the base link in the base link frame.
     */
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian_com(const Model<Scalar, nq>& model,
                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                        const TargetLink& target_link) {
        // Get the target link from the model
        const int target_link_idx = get_target_link_idx(model, target_link);
        Link<Scalar> current_link = model.links[target_link_idx];

        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTcBb = forward_kinematics_com(model, q, base_link.name, target_link).translation();

        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();
        while (current_link.idx != model.base_link_idx) {
            auto current_joint = current_link.joint;
            if (current_joint.idx != -1) {
                // Compute the transform between base {b} and the current link {i}
                auto Hbi = forward_kinematics(model, q, base_link.name, current_link.name);
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
            current_link = model.links[current_link.parent];
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
     * @tparam TargetLink Type of source_link parameter, which can be int or std::string.
     * @return The centre of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Matrix<Scalar, 3, 1> centre_of_mass(const Model<Scalar, nq>& model,
                                               const Eigen::Matrix<Scalar, nq, 1>& q,
                                               const TargetLink& source_link) {
        // For each link in the model, compute the transform from the source link to the CoM of the link
        Scalar total_mass                = 0;
        Eigen::Matrix<Scalar, 3, 1> rISs = Eigen::Matrix<Scalar, 3, 1>::Zero();
        for (auto link : model.links) {
            // Compute the transform from the source link to the CoM of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = forward_kinematics_com(model, q, source_link, link.name);
            // Compute the centre of mass of the link
            rISs = rISs + Hsc.translation() * link.mass;
            // Add the links mass to the total mass
            total_mass += link.mass;
        }
        // Compute the centre of mass  from the source link
        return rISs / total_mass;
    }

}  // namespace tinyrobotics

#endif
