#ifndef RML_JOINT_HPP
#define RML_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RML {

    /// @brief The types of joints that can be used in the model
    enum class JointType {
        UNKNOWN,     ///< Unknown joint type
        REVOLUTE,    ///< Revolute joint
        CONTINUOUS,  ///< Continuous joint
        PRISMATIC,   ///< Prismatic joint
        FLOATING,    ///< Floating joint
        PLANAR,      ///< Planar joint
        FIXED        ///< Fixed joint
    };

    /**
     * @brief Defines a joint in a robot model.
     *
     * @details A joint is used to connect two links in the model and defines how the child link can move
     * relative to the parent link.
     *
     * @tparam Scalar The scalar type of the joint
     */
    template <typename Scalar>
    struct Joint {

        /// @brief Index of the joint in the model's joint vector
        int joint_idx = -1;

        /// @brief Index of the joint in the model's configuration vector
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

        /// @brief Name of the parent link
        std::string parent_link_name = "";

        /// @brief Name of the child link
        std::string child_link_name = "";

        /// @brief Spatial transformation matrix from the parent to the child link
        Eigen::Matrix<Scalar, 6, 6> X = {};

        /**
         * @brief Casts the joint to a new scalar type.
         *
         * @details This function creates a new Joint object with the same properties as this joint, but with
         * a different scalar type.
         *
         * @tparam NewScalar The new scalar type for the joint
         * @return The new joint object with the same properties as this joint, but with a different
         * scalar type.
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
