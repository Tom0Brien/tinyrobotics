#ifndef RML_LINK_HPP
#define RML_LINK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Common.hpp"
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
         * @brief Construct a new Link from a URDF file description.
         * @param xml The XML element containing the link description
         */
        static std::shared_ptr<Link<Scalar>> fromXml(TiXmlElement* xml) {

            std::shared_ptr<Link<Scalar>> link = std::make_shared<Link<Scalar>>();

            const char* name_char = xml->Attribute("name");
            if (name_char != nullptr) {
                link->name = std::string(name_char);
            }
            else {
                std::ostringstream error_msg;
                error_msg << "Error! Link without a name attribute detected!";
                throw std::runtime_error(error_msg.str());
            }


            TiXmlElement* i = xml->FirstChildElement("inertial");
            if (i != nullptr) {
                // ************************ Add the centre of mass to the link ************************
                TiXmlElement* o = i->FirstChildElement("origin");
                if (o != nullptr) {
                    link->centre_of_mass = transform_from_xml<Scalar>(o);
                }
                // ************************ Add the mass to the link ************************
                TiXmlElement* mass_xml = i->FirstChildElement("mass");
                if (mass_xml != nullptr) {
                    if (mass_xml->Attribute("value") != nullptr) {
                        try {
                            link->mass = std::stod(mass_xml->Attribute("value"));
                        }
                        catch (std::invalid_argument& e) {
                            std::ostringstream error_msg;
                            error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                                      << "': inertial mass [" << mass_xml->Attribute("value")
                                      << "] is not a valid double: " << e.what() << "!";
                            throw std::runtime_error(error_msg.str());
                        }
                    }
                    else {
                        std::ostringstream error_msg;
                        error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                                  << "' <mass> element must have a value attribute!";
                        throw std::runtime_error(error_msg.str());
                    }
                }
                else {
                    std::ostringstream error_msg;
                    error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                              << "' inertial element must have a <mass> element!";
                    throw std::runtime_error(error_msg.str());
                }
            }
            // ************************ Add the inertia to the link ************************
            TiXmlElement* inertia_xml = i->FirstChildElement("inertia");
            if (inertia_xml != nullptr) {
                Eigen::Matrix<Scalar, 6, 1> inertia;
                if (inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz")
                    && inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz")
                    && inertia_xml->Attribute("izz")) {
                    try {
                        inertia(0)    = std::stod(inertia_xml->Attribute("ixx"));
                        inertia(1)    = std::stod(inertia_xml->Attribute("iyy"));
                        inertia(2)    = std::stod(inertia_xml->Attribute("izz"));
                        inertia(3)    = std::stod(inertia_xml->Attribute("iyz"));
                        inertia(4)    = std::stod(inertia_xml->Attribute("ixz"));
                        inertia(5)    = std::stod(inertia_xml->Attribute("ixy"));
                        link->inertia = inertia;
                    }
                    catch (std::invalid_argument& e) {
                        std::ostringstream error_msg;
                        error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                                  << "Inertial: one of the inertia elements is not a valid double:"
                                  << " ixx [" << inertia_xml->Attribute("ixx") << "]"
                                  << " ixy [" << inertia_xml->Attribute("ixy") << "]"
                                  << " ixz [" << inertia_xml->Attribute("ixz") << "]"
                                  << " iyy [" << inertia_xml->Attribute("iyy") << "]"
                                  << " iyz [" << inertia_xml->Attribute("iyz") << "]"
                                  << " izz [" << inertia_xml->Attribute("izz") << "]\n\n"
                                  << e.what();
                        throw std::runtime_error(error_msg.str());
                    }
                }
                else {
                    std::ostringstream error_msg;
                    error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                              << "' <inertia> element must have ixx,ixy,ixz,iyy,iyz,izz attributes!";
                    throw std::runtime_error(error_msg.str());
                }
            }
            else {
                std::ostringstream error_msg;
                error_msg << "Error while parsing link '" << link->get_parent_link_name(i)
                          << "' inertial element must have a <inertia> element!";
                throw std::runtime_error(error_msg.str());
            }

            return link;
        }

        /**
         * @brief Get the name of the link's parent link.
         * @param xml The XML element containing the link description
         * @return The name of the link's parent link.
         */
        const char* get_parent_link_name(TiXmlElement* c) {
            TiXmlElement* e = c->Parent()->ToElement();
            while (e->Parent() != nullptr) {
                if (e->ValueStr() == "link") {
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
