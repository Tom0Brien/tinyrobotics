#ifndef TR_LINK_HPP
#define TR_LINK_HPP

#include "joint.hpp"

/** \file link.hpp
 * @brief Contains struct for representing a joint in a tinyrobotics model.
 */
namespace tinyrobotics {

    /**
     * @brief Represents a link in a tinyrobotics model.
     * @tparam Scalar Scalar type of the link.
     */
    template <typename Scalar>
    struct Link {

        /// @brief Name of the link.
        std::string name = "";

        /// @brief Index of the link in the model's link vector.
        int idx = -1;

        /// @brief Index of the link's parent link in the model's link vector.
        int parent = -1;

        /// @brief Vector of child link indices in the model's link vector.
        std::vector<int> child_links = {};

        /// @brief Center of mass of the link.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> center_of_mass =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Inertia matrix of the link [kg m^2].
        Eigen::Matrix<Scalar, 3, 3> inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();

        /// @brief Mass of the link [kg].
        Scalar mass = 0;

        /// @brief Joint connecting the link to its parent link.
        Joint<Scalar> joint;

        // @brief Spatial inertia matrix of the link.
        Eigen::Matrix<Scalar, 6, 6> I = {};

        /**
         * @brief Add a child link index to the list of child links.
         * @param child_link_idx The index of the child link to add.
         */
        void add_child_link_idx(const int child_link_idx) {
            child_links.push_back(child_link_idx);
        }

        /**
         * @brief Casts the link to a new scalar type.
         * @tparam NewScalar scalar type to cast the link to.
         * @return Link with new scalar type.
         */
        template <typename NewScalar>
        Link<NewScalar> cast() {
            Link<NewScalar> new_link = Link<NewScalar>();
            new_link.name            = name;
            new_link.idx             = idx;
            new_link.parent          = parent;
            new_link.child_links     = child_links;
            new_link.center_of_mass  = center_of_mass.template cast<NewScalar>();
            new_link.inertia         = inertia.template cast<NewScalar>();
            new_link.mass            = NewScalar(mass);
            new_link.joint           = joint.template cast<NewScalar>();
            new_link.I               = I.template cast<NewScalar>();
            return new_link;
        }
    };
}  // namespace tinyrobotics
#endif
