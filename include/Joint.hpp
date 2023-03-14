#ifndef TR_JOINT_HPP
#define TR_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tr {

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
        Eigen::Matrix<Scalar, 6, 6> X = {};

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
}  // namespace tr
#endif
