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

        /// @brief Name of link
        std::string name = "";

        /// @brief Index of link in the model link vector
        int link_idx = -1;

        /// @brief Index of links parent link in the models link vector
        int parent = -1;

        /// @brief List of child link indices in the models link vector.
        std::vector<int> child_links;

        /// @brief Links centre of mass.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> centre_of_mass =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Links inertia matrix [kg m^2].
        Eigen::Matrix<Scalar, 3, 3> inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();

        /// @brief Mass of link [kg].
        Scalar mass = 0;

        /// @brief Links joint
        Joint<Scalar> joint;

        // @brief Spatial inertia matrices
        Eigen::Matrix<Scalar, 6, 6> I = {};

        /**
         * @brief Add child link index
         *
         */
        void add_child_link_idx(const int child_link_idx) {
            this->child_links.push_back(child_link_idx);
        }

        /**
         * @brief Cast to NewScalar type
         */
        template <typename NewScalar>
        Link<NewScalar> cast() {
            Link<NewScalar> new_link = Link<NewScalar>();
            new_link.name            = name;
            new_link.link_idx        = link_idx;
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
