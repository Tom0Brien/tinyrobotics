#ifndef TR_KINEMATICS_HPP
#define TR_KINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "math.hpp"
#include "model.hpp"

/** \file kinematics.hpp
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
    int get_link_idx(const Model<Scalar, nq>& model, const TargetLink& target_link) {
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
     * @return Stores the transform to all the links in model.forward_kinematics.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics(
        Model<Scalar, nq>& model,
        const Eigen::Matrix<Scalar, nq, 1>& q) {
        model.forward_kinematics.resize(model.links.size(), Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        for (auto const link : model.links) {
            model.forward_kinematics[link.idx] = link.joint.parent_transform;
            if (link.joint.idx != -1) {
                model.forward_kinematics[link.idx] =
                    model.forward_kinematics[link.idx] * link.joint.get_joint_transform(q[link.joint.idx]);
            }
            if (link.parent != -1) {
                model.forward_kinematics[link.idx] =
                    model.forward_kinematics[link.parent] * model.forward_kinematics[link.idx];
            }
        }
        return model.forward_kinematics;
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
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const TargetLink& target_link) {
        Link<Scalar> current_link = model.links[get_link_idx(model, target_link)];

        // Build kinematic tree from base {b} to target {t} link
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htb = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        while (current_link.idx != model.base_link_idx) {
            // Apply joint transform if joint is actuatable
            if (current_link.joint.idx != -1) {
                Htb = Htb * current_link.joint.get_joint_transform(q[current_link.joint.idx]).inverse();
            }
            Htb          = Htb * current_link.joint.parent_transform.inverse();
            current_link = model.links[current_link.parent];
        }
        return Htb.inverse();
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
    template <typename Scalar, int nq, typename TargetLink, typename SourceLink>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics(const Model<Scalar, nq>& model,
                                                                    const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                    const TargetLink& target_link,
                                                                    const SourceLink& source_link) {
        return forward_kinematics(model, q, get_link_idx(model, source_link)).inverse()
               * forward_kinematics(model, q, get_link_idx(model, target_link));
    }

    /**
     * @brief Computes the transform between center of mass of each link and the source link.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @return Vector of homogeneous transforms between the center of mass of each link and the source link.
     */
    template <typename Scalar, int nq>
    std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics_com(
        Model<Scalar, nq>& model,
        const Eigen::Matrix<Scalar, nq, 1>& q) {
        // Compute forward kinematics for all the links
        forward_kinematics(model, q);
        model.forward_kinematics_com.resize(model.links.size(),
                                            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        // Apply center of mass transform for each link
        for (auto const link : model.links) {
            model.forward_kinematics_com[link.idx] =
                model.forward_kinematics[link.idx] * model.links[link.idx].center_of_mass;
        }
        return model.forward_kinematics_com;
    }

    /**
     * @brief Computes the transform between source link and target center of mass. The transform converts points in the
     * target links center of mass frame into the source link frame.
     * @param model tinyrobotics model.
     * @param q Joint configuration of the robot.
     * @param target_link Target link, which can be an integer (index) or a string (name).
     * @param source_link Source link, which can be an integer (index) or a string (name).
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @return Homogeneous transform from source link to target link center of mass.
     */
    template <typename Scalar, int nq, typename TargetLink, typename SourceLink = int>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> forward_kinematics_com(const Model<Scalar, nq>& model,
                                                                        const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                        const TargetLink& target_link,
                                                                        const SourceLink& source_link = 0) {
        // Apply center of mass transform to forward kinematics between source and target link
        return forward_kinematics(model, q, target_link, source_link)
               * model.links[get_link_idx(model, target_link)].center_of_mass;
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
    template <typename Scalar, int nq, typename TargetLink>
    Eigen::Matrix<Scalar, 6, nq> jacobian(Model<Scalar, nq>& model,
                                          const Eigen::Matrix<Scalar, nq, 1>& q,
                                          const TargetLink& target_link) {
        // Compute forward kinematics for all the links
        forward_kinematics(model, q);

        model.J.setZero();
        Eigen::Matrix<Scalar, 3, 1> zIBb;
        Eigen::Matrix<Scalar, 3, 1> rIBb;
        Link<Scalar> current_link        = model.links[get_link_idx(model, target_link)];
        Eigen::Matrix<Scalar, 3, 1> rTBb = model.forward_kinematics[current_link.idx].translation();

        // Compute the jacobian
        while (current_link.idx != model.base_link_idx) {
            if (current_link.joint.idx != -1) {
                zIBb = model.forward_kinematics[current_link.idx].linear() * current_link.joint.axis;
                rIBb = model.forward_kinematics[current_link.idx].translation();
                if (current_link.joint.type == JointType::PRISMATIC) {
                    model.J.block(0, current_link.joint.idx, 3, 1) = zIBb;
                    model.J.block(3, current_link.joint.idx, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_link.joint.type == JointType::REVOLUTE) {
                    model.J.block(0, current_link.joint.idx, 3, 1) = (zIBb).cross(rTBb - rIBb);
                    model.J.block(3, current_link.joint.idx, 3, 1) = zIBb;
                }
            }
            // Move up the tree to parent towards the base
            current_link = model.links[current_link.parent];
        }
        return model.J;
    }

    /**
     * @brief Computes the center of mass expressed in source link frame.
     * @param model tinyrobotics model.
     * @param q The configuration vector of the robot model.
     * @param source_link Source link, which can be an integer (index) or a string (name), from which the center of mass
     * is expressed.
     * @tparam Scalar type of the tinyrobotics model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
     * @return The center of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq, typename SourceLink = int>
    Eigen::Matrix<Scalar, 3, 1> center_of_mass(Model<Scalar, nq>& model,
                                               const Eigen::Matrix<Scalar, nq, 1>& q,
                                               const SourceLink& source_link = 0) {
        forward_kinematics_com(model, q);
        model.center_of_mass.setZero();
        for (auto link : model.links) {
            model.center_of_mass += model.forward_kinematics_com[link.idx].translation() * link.mass;
        }
        model.center_of_mass /= model.mass;
        return model.center_of_mass;
    }

}  // namespace tinyrobotics

#endif
