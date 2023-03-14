#ifndef TR_FORWARDKINEMATICS_HPP
#define TR_FORWARDKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Math.hpp"
#include "Model.hpp"

namespace tr {

    /**
     * @brief Computes the transform to the target from the base link.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link_idx Index of the link to which the transform is computed.
     * @param inverse_transform Optional: If true, returns the transform to the base from the target link.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const int& target_link_idx,
                                                                    const bool inverse_transform = false) {
        // Get the target link
        Link<Scalar> current_link = model.links[target_link_idx];

        // Check if the link is within the kinematic tree
        if (current_link.idx == -1) {
            std::string error_msg = "Error: Link " + std::to_string(target_link_idx) + " not found.";
            throw std::runtime_error(error_msg);
        }

        // Return identity transform if the link is itself the base link
        if (current_link.idx == model.base_link_idx) {
            return Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        }

        // Build kinematic tree from target_link {t} to base {b}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htb = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        while (current_link.idx != model.links[model.base_link_idx].idx) {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> H = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
            auto current_joint                             = current_link.joint;
            if (current_joint.type == JointType::REVOLUTE) {
                // Rotate by q_current around axis
                H.linear() =
                    Eigen::AngleAxis<Scalar>(Scalar(q(current_joint.idx)), current_joint.axis).toRotationMatrix();
            }
            else if (current_joint.type == JointType::PRISMATIC) {
                // Translate by q_current along axis
                H.translation() = current_joint.axis * Scalar(q(current_joint.idx));
            }
            Htb = Htb * H.inverse();
            // Apply inverse joint transform as we are going back up tree
            Htb = Htb * current_joint.parent_transform.inverse();
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
     * @brief Computes the transform between base and target link.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link_idx Name of the link to which the transform is computed.
     * @param inverse_transform Optional: If true, returns the transform to the base from the target link.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const std::string& target_link_name,
                                                                    const bool inverse_transform = false) {
        // Get the target link by name
        Link<Scalar> target_link = model.get_link(target_link_name);
        return forward_kinematics(model, q, target_link.idx, inverse_transform);
    }

    /**
     * @brief Computes the homogeneous transform from a source to target link.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_name Name of source {s} link from which the transform is computed.
     * @param target_link_name Name of target {t} link to which the transform is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform from source {s} to target {t} link.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const std::string& source_link_name,
                                                                    const std::string& target_link_name) {
        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsb = forward_kinematics(model, q, source_link_name, true);
        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbt = forward_kinematics(model, q, target_link_name);
        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the homogeneous transform from a source to target link.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_idx Index of source {s} link from which the transform is computed.
     * @param target_link_idx Index of target {t} link to which the transform is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform from source {s} to target {t} link.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const int& source_link_idx,
                                                                    const int& target_link_idx) {
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
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_name Name of source {s} link from which the transform is computed.
     * @param target_link_name Name of target {t} link to which the transform is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform from source link {s} to target link centre of mass {c}.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(const Model<Scalar, nq>& model,
                                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                        const std::string& source_link_name,
                                                                        const std::string& target_link_name) {
        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = Hst * model.get_link(target_link_name).centre_of_mass;
        // Return transform from source {s} to CoM {c}
        return Hsc;
    }

    /**
     * @brief Computes the transform between source link {s} and target {t} centre of mass (CoM) {c}.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_idx Index of source {s} link from which the transform is computed.
     * @param target_link_idx Index of target {t} link to which the transform is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Homogeneous transform from source link {s} to target link centre of mass {c}.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(const Model<Scalar, nq>& model,
                                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                        const int& source_link_idx,
                                                                        const int& target_link_idx) {
        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = Hst * model.links[target_link_idx].centre_of_mass;
        return Hsc;
    }

    /**
     * @brief Computes the translation of the target link from the source link in the source link frame.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_name Name of source {s} link from which the translation is computed.
     * @param target_link_name Name of target {t} link to which the translation is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Translation of the target link from the source link in the source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> translation(const Model<Scalar, nq>& model,
                                            const Eigen::Matrix<Scalar, nq, 1>& q,
                                            const std::string& source_link_name,
                                            const std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the translation of the target link from the source link in the source link frame.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_idx Index of source {s} link from which the translation is computed.
     * @param target_link_idx Index of target {t} link to which the translation is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Translation of the target link from the source link in the source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> translation(const Model<Scalar, nq>& model,
                                            const Eigen::Matrix<Scalar, nq, 1>& q,
                                            const int& source_link_idx,
                                            const int& target_link_idx) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the rotation matrix between the target link and the source link in the source link frame.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param source_link_name Name of source {s} link from which the rotation matrix is computed.
     * @param target_link_name Name of target {t} link to which the rotation matrix is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Rotation matrix between the target link and the source link in the source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 3> rotation(const Model<Scalar, nq>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         const std::string& source_link_name,
                                         const std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        return Hst.linear();
    }

    /**
     * @brief Computes the geometric jacobian of the target link from the base link in the base link frame.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link_idx Index of target {t} link to which the geometric jacobian is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The geometric jacobian of the target link from the base link in the base link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian(const Model<Scalar, nq>& model,
                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                    const int& target_link_idx) {
        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        // Get the target link from the model
        Link<Scalar> current_link = model.links[target_link_idx];

        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTBb = translation(model, q, base_link.idx, current_link.idx);

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
     * @brief Computes the geometric jacobian relative to the base for the specified target link.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link_name Name of target {t} link to which the geometric jacobian is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The geometric jacobian relative to the base for the specified target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian(const Model<Scalar, nq>& model,
                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                    const std::string& target_link_name) {
        // Get the target link by name
        Link<Scalar> target_link = model.get_link(target_link_name);
        return geometric_jacobian(model, q, target_link.idx);
    }

    /**
     * @brief Computes the geometric jacobian relative to the base for the specified target link's center of mass.
     * @param model Tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link_name {n} The link to which the transform is computed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian_com(const Model<Scalar, nq>& model,
                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                        const std::string& target_link_name) {
        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        // Get the target link from the model
        Link<Scalar> current_link = model.get_link(target_link_name);

        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTcBb =
            forward_kinematics_com(model, q, base_link.name, target_link_name).translation();

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
     * @param model Tinyrobotics model.
     * @param q The configuration vector of the robot model.
     * @param source_link_name Name of source {s} link frame from which the centre of mass is expressed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return The centre of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> centre_of_mass(const Model<Scalar, nq>& model,
                                               const Eigen::Matrix<Scalar, nq, 1>& q,
                                               const std::string& source_link_name) {
        // For each link in the model, compute the transform from the source link to the CoM of the link
        Scalar total_mass                = 0;
        Eigen::Matrix<Scalar, 3, 1> rISs = Eigen::Matrix<Scalar, 3, 1>::Zero();
        for (auto link : model.links) {
            // Compute the transform from the source link to the CoM of the link
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc =
                forward_kinematics_com(model, q, source_link_name, link->name);
            // Compute the centre of mass of the link
            rISs = rISs + Hsc.translation() * link->mass;
            // Add the links mass to the total mass
            total_mass += link->mass;
        }
        // Compute the centre of mass  from the source link
        return rISs / total_mass;
    }

}  // namespace tr

#endif
