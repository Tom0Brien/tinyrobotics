#ifndef RML_JOINT_HPP
#define RML_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RML {

    /// @brief The types of joints
    enum class JointType { UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED };

    /**
     * @brief Defines how a link moves relative to an attachment point
     * @details
     * @param Scalar The scalar type of the joint
     */
    template <typename Scalar>
    struct Joint {

        /// @brief Index of joint in the model joint vector
        int joint_idx = -1;

        /// @brief Index of joint in the models configuration vector
        int q_idx = -1;

        /// @brief Name of the joint
        std::string name = "";

        /// @brief Type of the joint
        JointType type = JointType::UNKNOWN;

        /// @brief Axis of motion for the joint
        Eigen::Matrix<Scalar, 3, 1> axis = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Transform to the parent link
        Eigen::Transform<Scalar, 3, Eigen::Isometry> parent_transform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform to the child link
        Eigen::Transform<Scalar, 3, Eigen::Isometry> child_transform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Parent link name
        std::string parent_link_name = "";

        /// @brief Child link name
        std::string child_link_name = "";

        /// @brief Spatial transformation
        Eigen::Matrix<Scalar, 6, 6> X = {};

        /**
         * @brief Casts to NewScalar type
         */
        template <typename NewScalar>
        Joint<NewScalar> cast() const {
            Joint<NewScalar> new_joint = Joint<NewScalar>();
            new_joint.joint_idx        = joint_idx;
            new_joint.q_idx            = q_idx;
            new_joint.name             = name;
            new_joint.type             = type;
            new_joint.axis             = axis.template cast<NewScalar>();
            new_joint.parent_transform = parent_transform.template cast<NewScalar>();
            new_joint.child_transform  = child_transform.template cast<NewScalar>();
            new_joint.parent_link_name = parent_link_name;
            new_joint.child_link_name  = child_link_name;
            return new_joint;
        }
    };
}  // namespace RML

#endif
