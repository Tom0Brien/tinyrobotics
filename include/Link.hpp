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

        /// @brief Unique ID of the link
        int id = -1;

        /// @brief The links joint.
        std::shared_ptr<Joint<Scalar>> joint = nullptr;

        /// @brief The links centre of mass.
        Eigen::Transform<Scalar, 3, Eigen::Affine> centre_of_mass =
            Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();

        /// @brief The links inertia matrix [kg m^2].
        Eigen::Matrix<Scalar, 6, 1> inertia = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /// @brief The links mass [kg].
        Scalar mass = 0;

        /// @brief The links parent link.
        std::shared_ptr<Link<Scalar>> parent_link = nullptr;

        /// @brief The list of child links.
        std::vector<std::shared_ptr<Link<Scalar>>> child_links;

        /// @brief The list of child joints.
        std::vector<std::shared_ptr<Joint<Scalar>>> child_joints;

        /**
         * @brief Get the name of the link's parent link.
         * @param xml The XML element containing the link description
         * @return The name of the link's parent link.
         */
        const char* get_parent_link_name(tinyxml2::XMLElement* c) {
            tinyxml2::XMLElement* e = c->Parent()->ToElement();
            while (e->Parent() != nullptr) {
                if (e->Value() == "link") {
                    break;
                }
                e = e->Parent()->ToElement();
            }
            return e->Attribute("name");
        }

        /**
         * @brief Add child link
         *
         */

        void add_child_link(const std::shared_ptr<Link<Scalar>> child_link) {
            this->child_links.push_back(child_link);
        }

        /**
         * @brief Add child joint
         */
        void add_child_joint(const std::shared_ptr<Joint<Scalar>> child_joint) {
            this->child_joints.push_back(child_joint);
        }

        /**
         * @brief Cast to NewScalar type
         */
        template <typename NewScalar>
        std::shared_ptr<Link<NewScalar>> cast() {
            std::shared_ptr<Link<NewScalar>> new_link = std::make_shared<Link<NewScalar>>();
            new_link->name                            = name;
            new_link->id                              = id;
            new_link->centre_of_mass                  = centre_of_mass.template cast<NewScalar>();
            new_link->mass                            = mass;
            new_link->inertia                         = inertia.template cast<NewScalar>();
            return new_link;
        }
    };
}  // namespace RML

#endif
