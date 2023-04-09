#ifndef TR_JOINT_HPP
#define TR_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

/** \file joint.hpp
 * @brief Contains struct for representing a joint in a tinyrobotics model.
 */
namespace tinyrobotics {

    /// @brief The types of joints.
    enum class JointType {
        UNKNOWN,     ///< Unknown joint type
        REVOLUTE,    ///< Revolute joint, can rotate about an axis
        CONTINUOUS,  ///< Continuous joint
        PRISMATIC,   ///< Prismatic joint, can translate along an axis
        FLOATING,    ///< Floating joint, can translate and rotate about any axis
        PLANAR,      ///< Planar joint
        FIXED        ///< Fixed joint, cannot move
    };

    /**
     * @brief Represents a joint in a tinyrobotics model which connects two links.
     * @tparam Scalar The scalar type of the joint
     */
    template <typename Scalar>
    struct Joint {

        /// @brief Index of the joint in the tinyrobotics model's joint vector.
        int joint_id = -1;

        /// @brief Index of the joint in the tinyrobotics model's configuration vector.
        int idx = -1;

        /// @brief Name of the joint.
        std::string name = "";

        /// @brief Type of the joint.
        JointType type = JointType::UNKNOWN;

        /// @brief Axis of motion for the joint.
        Eigen::Matrix<Scalar, 3, 1> axis = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Homogeneous transform to the parent link.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> parent_transform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Homogeneous transform to the child link.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> child_transform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Name of the joints parent link.
        std::string parent_link_name = "";

        /// @brief Name of the joints child link.
        std::string child_link_name = "";

        /// @brief Spatial transformation matrix from the parent to the child link.
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();

        /// @brief Spatial axis
        Eigen::Matrix<Scalar, 6, 1> S = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /**
         * @brief Get joint type as a string.
         * @param joint_type The joint type to convert to a string.
         * @return String representation of the joint type.
         */
        std::string get_type() const {
            switch (type) {
                case JointType::UNKNOWN: return "Unknown";
                case JointType::REVOLUTE: return "Revolute";
                case JointType::CONTINUOUS: return "Continuous";
                case JointType::PRISMATIC: return "Prismatic";
                case JointType::FLOATING: return "Floating";
                case JointType::PLANAR: return "Planar";
                case JointType::FIXED: return "Fixed";
                default: return "Invalid";
            }
        }

        /**
         * @brief Compute the joint transform.
         * @param q The joint position variable.
         * @return Homogeneous transform of the joint from its home position to its transformed position.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_joint_transform(const Scalar& q) const {
            Eigen::Transform<Scalar, 3, Eigen::Isometry> T = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
            switch (type) {
                case JointType::REVOLUTE: {
                    T.linear() = Eigen::AngleAxis<Scalar>(q, Eigen::Matrix<Scalar, 3, 1>(axis[0], axis[1], axis[2]))
                                     .toRotationMatrix();
                    return T;
                    break;
                }
                case JointType::PRISMATIC: {
                    T.translation() = q * Eigen::Matrix<Scalar, 3, 1>(axis[0], axis[1], axis[2]);
                    return T;
                    break;
                }
                case JointType::FIXED: {
                    return T;
                    break;
                }
                default: {
                    std::cout << "Joint type not supported." << std::endl;
                    break;
                }
            }
            return T;
        }

        /**
         * @brief Compute the transform from parent to child.
         * @param q The joint position variable.
         * @return Homogeneous transform from parent to child.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_parent_to_child_transform(const Scalar& q) {
            return parent_transform * get_joint_transform(q) * child_transform;
        }

        /**
         * @brief Casts the joint to a new scalar type.
         * @tparam NewScalar Scalar type to cast the joint to.
         * @return Joint with new scalar type.
         */
        template <typename NewScalar>
        Joint<NewScalar> cast() const {
            Joint<NewScalar> new_joint = Joint<NewScalar>();
            new_joint.joint_id         = joint_id;
            new_joint.idx              = idx;
            new_joint.name             = name;
            new_joint.type             = type;
            new_joint.axis             = axis.template cast<NewScalar>();
            new_joint.parent_transform = parent_transform.template cast<NewScalar>();
            new_joint.child_transform  = child_transform.template cast<NewScalar>();
            new_joint.parent_link_name = parent_link_name;
            new_joint.child_link_name  = child_link_name;
            new_joint.X                = X.template cast<NewScalar>();
            return new_joint;
        }
    };


}  // namespace tinyrobotics
#endif
