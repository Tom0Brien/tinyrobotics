#ifndef RML_LINK_HPP
#define RML_LINK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Joint.hpp"

namespace RML {

    /**
     * @brief Represents a link in a robot model.
     * @tparam Scalar The scalar type of the link.
     */
    template <typename Scalar>
    struct Link {

        /// @brief Name of the link.
        std::string name = "";

        /// @brief Index of the link in the model's link vector.
        int idx = -1;

        /// @brief Index of the link's parent link in the model's link vector.
        int parent = -1;

        /// @brief List of child link indices in the model's link vector.
        std::vector<int> child_links;

        /// @brief The center of mass of the link.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> centre_of_mass =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief The inertia matrix of the link [kg m^2].
        Eigen::Matrix<Scalar, 3, 3> inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();

        /// @brief The mass of the link [kg].
        Scalar mass = 0;

        /// @brief The link's joint.
        Joint<Scalar> joint;

        // @brief The link's spatial inertia matrix.
        Eigen::Matrix<Scalar, 6, 6> I = {};

        /**
         * @brief Add a child link index to the list of child links.
         * @param child_link_idx The index of the child link to add.
         */
        void add_child_link_idx(const int child_link_idx) {
            this->child_links.push_back(child_link_idx);
        }

        /**
         * @brief Cast the link to a new scalar type.
         * @tparam NewScalar The scalar type to cast the link to.
         * @return The link with the new scalar type.
         */
        template <typename NewScalar>
        Link<NewScalar> cast() {
            Link<NewScalar> new_link = Link<NewScalar>();
            new_link.name            = name;
            new_link.idx             = idx;
            new_link.joint           = joint.template cast<NewScalar>();
            new_link.centre_of_mass  = centre_of_mass.template cast<NewScalar>();
            new_link.mass            = NewScalar(mass);
            new_link.inertia         = inertia.template cast<NewScalar>();
            new_link.parent          = parent;
            new_link.child_links     = child_links;
            return new_link;
        }
    };
}  // namespace RML

#endif
