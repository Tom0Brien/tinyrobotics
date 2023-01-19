#ifndef RML_FORWARDKINEMATICS_HPP
#define RML_FORWARDKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Math.hpp"
#include "Model.hpp"

namespace RML {

    /**
     * @brief Computes the transform between base link to target.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_idx {s} The idx of the link to which the transform is computed.
     * @return The transform between the base and the target link
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const int& target_link_idx) {

        // Get the target link
        Link<Scalar> current_link = model.links[target_link_idx];

        // Check if the link is within the kinematic tree
        if (current_link.link_idx == -1) {
            std::string error_msg = "Error: Link " + std::to_string(target_link_idx) + " not found.";
            throw std::runtime_error(error_msg);
        }

        // Return identity transform if the link is the base link
        if (current_link.name == model.links[model.base_link_idx].name) {
            return Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        }

        // Build kinematic tree from target_link {t} to base {b}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htb = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        while (current_link.name != model.links[model.base_link_idx].name) {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> H = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
            auto current_joint                             = model.joints[current_link.joint_idx];
            if (current_joint.type == JointType::REVOLUTE) {
                Scalar q_current = q(current_joint.q_idx);
                // Rotate by q_current around axis
                H.linear() = Eigen::AngleAxis<Scalar>(q_current, current_joint.axis).toRotationMatrix();
            }
            else if (current_joint.type == JointType::PRISMATIC) {
                Scalar q_current = q(current_joint.q_idx);
                // Translate by q_current along axis
                H.translation() = current_joint.axis * q_current;
            }
            Htb = Htb * RML::inv(H);
            // Apply inverse joint transform as we are going back up tree
            Htb = Htb * RML::inv(current_joint.parent_transform);
            // Move up the tree to parent
            current_link = model.links[current_link.parent_link_idx];
        }

        // Return transform from base {b} to target {t}
        return RML::inv(Htb);
    }

    /**
     * @brief Computes the transform between base link to target.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {s} The link from which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const std::string& target_link_name) {
        // Get the target link
        Link<Scalar> current_link = model.get_link(target_link_name);

        // Return transform from base {b} to target {t}
        return forward_kinematics(model, q, current_link.link_idx);
    }

    /**
     * @brief Computes the transform between two links.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const std::string& source_link_name,
                                                                    const std::string& target_link_name) {
        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbs = forward_kinematics(model, q, source_link_name);
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsb = RML::inv(Hbs);

        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbt = forward_kinematics(model, q, target_link_name);

        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the transform between two links.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_idx {s} The link from which the transform is computed.
     * @param target_link_idx {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const int& source_link_idx,
                                                                    const int& target_link_idx) {
        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbs = forward_kinematics(model, q, source_link_idx);
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsb = RML::inv(Hbs);

        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hbt = forward_kinematics(model, q, target_link_idx);

        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the transform between source link {s} and target {t} centre of mass (CoM) {c}.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(Model<Scalar, nq>& model,
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
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(Model<Scalar, nq>& model,
                                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                        const int source_link_idx,
                                                                        const int target_link_idx) {
        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);

        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hsc = Hst * model.links[target_link_idx].centre_of_mass;

        // Return transform from source {s} to CoM {c}
        return Hsc;
    }

    /**
     * @brief Computes the position of the target link in the source link frame.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> position(Model<Scalar, nq>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         std::string& source_link_name,
                                         std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the position of the target link in the source link frame.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_idx {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> position(Model<Scalar, nq>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         int& source_link_idx,
                                         int& target_link_idx) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_idx, target_link_idx);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the rotation matrix to the target link in the source link frame
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The rotation matrix between the source and target link
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 3> rotation(Model<Scalar, nq>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         std::string& source_link_name,
                                         std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        return Hst.linear();
    }

    /**
     * @brief Computes the geometric Jacobian between the base and the target link
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {n} The link to which the transform is computed.
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian(Model<Scalar, nq>& model,
                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                    const std::string& target_link_name) {
        // Initialize the geometric jabobian matrix with zeros
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        // Get the target link from the model
        Link<Scalar> current_link = model.get_link(target_link_name);

        // Get the base link from the model
        auto base_link = model.links[model.base_link_idx];

        // Compute the displacement of the target link {t} in the base link frame {b}
        Eigen::Matrix<Scalar, 3, 1> rTBb = position(model, q, base_link.link_idx, current_link.link_idx);

        while (current_link.name != model.links[model.base_link_idx].name) {
            auto current_joint = model.joints[current_link.joint_idx];
            if (current_joint.q_idx != -1) {
                // Compute the transform between base {b} and the current link {i}
                auto Hbi = forward_kinematics(model, q, base_link.link_idx, current_link.link_idx);
                // Axis of the current joint rotated into the base frame {b}
                auto zIBb = Hbi.linear() * current_joint.axis;
                // Compute the displacement of the current link {i} from the base link frame {b}
                auto rIBb = Hbi.translation();
                if (current_joint.type == JointType::PRISMATIC) {
                    J.block(0, current_joint.q_idx, 3, 1) = zIBb;
                    J.block(3, current_joint.q_idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_joint.type == JointType::REVOLUTE) {
                    J.block(0, current_joint.q_idx, 3, 1) = (zIBb).cross(rTBb - rIBb);
                    J.block(3, current_joint.q_idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = model.links[current_link.parent_link_idx];
        }
        return J;
    }

    /**
     * @brief Computes the translational component of geometric Jacobian between the base and the target link
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {n} The link to which the transform is computed.
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, nq> Jv(Model<Scalar, nq>& model,
                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                    const std::string& target_link_name) {
        Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q, target_link_name);
        return J.block(0, 0, 3, nq);
    }

    /**
     * @brief Computes the rotational component of geometric Jacobian between the base and the target link
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {n} The link to which the transform is computed.
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, nq> Jw(Model<Scalar, nq>& model,
                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                    const std::string& target_link_name) {
        Eigen::Matrix<Scalar, 6, nq> J = geometric_jacobian(model, q, target_link_name);
        return J.block(3, 0, 3, nq);
    }

    /**
     * @brief Computes the geometric Jacobian between the base and the target link centre of mass
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {n} The link to which the transform is computed.
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian_com(Model<Scalar, nq>& model,
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

        while (current_link.name != model.links[model.base_link_idx].name) {
            auto current_joint = model.joints[current_link.joint_idx];
            if (current_joint.q_idx != -1) {
                // Compute the transform between base {b} and the current link {i}
                auto Hbi = forward_kinematics(model, q, base_link.name, current_link.name);
                // Axis of the current joint rotated into the base frame {b}
                auto zIBb = Hbi.linear() * current_joint.axis;
                // Compute the displacement of the current link {i} from the base link frame {b}
                auto rIBb = Hbi.translation();
                if (current_joint.type == JointType::PRISMATIC) {
                    J.block(0, current_joint.q_idx, 3, 1) = zIBb;
                    J.block(3, current_joint.q_idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_joint.type == JointType::REVOLUTE) {
                    J.block(0, current_joint.q_idx, 3, 1) = (zIBb).cross(rTcBb - rIBb);
                    J.block(3, current_joint.q_idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = model.links[current_link.parent_link_idx];
        }
        return J;
    }

    /**
     * @brief Computes the centre of mass expressed in source link frame.
     * @param model The robot model.
     * @param q The configuration vector of the robot model.
     * @param source_link_name The link from which the centre of mass position is computed.
     *
     * @return The centre of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> centre_of_mass(Model<Scalar, nq>& model,
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

}  // namespace RML

#endif
