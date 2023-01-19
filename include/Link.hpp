#ifndef RML_LINK_HPP
#define RML_LINK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Joint.hpp"

namespace RML {

    /**
     * @brief Represents a link in a robot model.
     * @details
     * @param Scalar Is the scalar type of the link
     */
    template <typename Scalar>
    struct Link {

        /// @brief Name of the link
        std::string name = "";

        /// @brief The index of the link in the model link vector
        int link_idx = -1;

        /// @brief The index of links joint in the model joint vector
        int joint_idx = -1;

        /// @brief The links centre of mass.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> centre_of_mass =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief The links inertia matrix [kg m^2].
        Eigen::Matrix<Scalar, 3, 3> inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();

        /// @brief The links mass [kg].
        Scalar mass = 0;

        /// @brief The index of the links parent link in the models link vector
        int parent_link_idx = -1;

        /// @brief The list of child link indices in the models link vector.
        std::vector<int> child_links;

        /// @brief The list of child joint indices in the models joint vector.
        std::vector<int> child_joints;

        /**
         * @brief Add child link index
         *
         */
        void add_child_link_idx(const int child_link_idx) {
            this->child_links.push_back(child_link_idx);
        }

        /**
         * @brief Add child joint index
         */
        void add_child_joint_idx(const int child_joint_idx) {
            this->child_joints.push_back(child_joint_idx);
        }

        /**
         * @brief Cast to NewScalar type
         */
        template <typename NewScalar>
        Link<NewScalar> cast() {
            Link<NewScalar> new_link = Link<NewScalar>();
            new_link.name            = name;
            new_link.link_idx        = link_idx;
            new_link.joint_idx       = joint_idx;
            new_link.centre_of_mass  = centre_of_mass.template cast<NewScalar>();
            new_link.mass            = NewScalar(mass);
            new_link.inertia         = inertia.template cast<NewScalar>();
            new_link.parent_link_idx = parent_link_idx;
            new_link.child_links     = child_links;
            new_link.child_joints    = child_joints;
            return new_link;
        }
    };
}  // namespace RML

#endif
