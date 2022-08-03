#ifndef RML_JOINT_HPP
#define RML_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "txml.h"
#include "Common.hpp"

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

        /// @brief The unique ID of the joint
        int id;

        /// @brief The q_index of the joint
        int q_index = -1;

        /// @brief The name of the joint
        std::string name = "";

        /// @brief The type of the joint
        JointType type = JointType::UNKNOWN;

        /// @brief The axis of motion for the joint
        Eigen::Matrix<Scalar, 3, 1> axis = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief The transform to the parent link
        Eigen::Transform<Scalar, 3, Eigen::Affine> parent_transform = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();

        /// @brief The parent link name
        std::string parent_link_name = "";

        /// @brief The transform to the child link
        Eigen::Transform<Scalar, 3, Eigen::Affine> child_transform = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();

        /// @brief The child link name
        std::string child_link_name = "";

        /**
         * @brief Construct a new Joint object from a URDF file description
         * @details
         * @param xml The XML element containing the joint description
         */
        static std::shared_ptr<Joint<Scalar>> fromXml(TiXmlElement* xml) {
            std::shared_ptr<Joint<Scalar>> joint= std::make_shared<Joint>();

            const char *name = xml->Attribute("name");
            if (name != NULL) {
                joint->name = std::string(name);;
            } else {
                std::ostringstream error_msg;
                error_msg << "Error while parsing model: unnamed joint found!";
                throw std::runtime_error(error_msg.str());
            }

            TiXmlElement *origin_xml = xml->FirstChildElement("origin");
            if (origin_xml != NULL) {
                    joint->parent_transform = transform_from_xml<Scalar>(origin_xml);
            }

            TiXmlElement *parent_xml = xml->FirstChildElement("parent");
            if (parent_xml != NULL) {
                const char *pname = parent_xml->Attribute("link");
                if (pname != NULL) {
                    joint->parent_link_name = std::string(pname);
                }
                // if no parent link name specified. this might be the root node
            }

            TiXmlElement *child_xml = xml->FirstChildElement("child");
            if (child_xml)
            {
                const char *pname = child_xml->Attribute("link");
                if (pname != NULL) {
                    joint->child_link_name = std::string(pname);
                }
            }

            const char* type_char = xml->Attribute("type");
            if (type_char == NULL) {
                std::ostringstream error_msg;
                error_msg << "Error! Joint " << joint->name
                        <<" has no type, check to see if it's a reference.";
                throw std::runtime_error(error_msg.str());
            }

            std::string type_str = type_char;
            if (type_str == "planar")
                joint->type = JointType::PLANAR;
            else if (type_str == "floating")
                joint->type = JointType::FLOATING;
            else if (type_str == "revolute")
                joint->type = JointType::REVOLUTE;
            else if (type_str == "continuous")
                joint->type = JointType::CONTINUOUS;
            else if (type_str == "prismatic")
                joint->type = JointType::PRISMATIC;
            else if (type_str == "fixed")
                joint->type = JointType::FIXED;
            else {
                std::ostringstream error_msg;
                error_msg << "Error! Joint '" << joint->name
                        <<"' has unknown type (" << type_str << ")!";
                throw std::runtime_error(error_msg.str());
            }

            if (joint->type != JointType::FLOATING && joint->type != JointType::FIXED)
            {
                TiXmlElement *axis_xml = xml->FirstChildElement("axis");
                if (axis_xml == NULL) {
                    Eigen::Matrix<Scalar, 3, 1> default_axis;
                    default_axis << 1, 0, 0;
                    joint->axis = default_axis;
                } else {
                    const char *xyz_char = axis_xml->Attribute("xyz");
                    if (xyz_char != NULL){
                        joint->axis = vec_from_string<Scalar>(std::string(xyz_char));
                    }
                }
            }

            // TiXmlElement *prop_xml = xml->FirstChildElement("dynamics");
            // if (prop_xml != NULL) {
            //     joint->dynamics = JointDynamics::fromXml(prop_xml);
            // }

            // TiXmlElement *limit_xml = xml->FirstChildElement("limit");
            // if (limit_xml != NULL) {
            //     joint->limits = JointLimits::fromXml(limit_xml);
            // }

            // TiXmlElement *safety_xml = xml->FirstChildElement("safety_controller");
            // if (safety_xml != NULL) {
            //     joint->safety = JointSafety::fromXml(safety_xml);
            // }

            // TiXmlElement *calibration_xml = xml->FirstChildElement("calibration");
            // if (calibration_xml != NULL) {
            //     joint->calibration = JointCalibration::fromXml(calibration_xml);
            // }

            // TiXmlElement *mimic_xml = xml->FirstChildElement("mimic");
            // if (mimic_xml != NULL) {
            //     joint->mimic = JointMimic::fromXml(mimic_xml);
            // }

            return joint;
        }

        /**
         * @brief Cast to NewScalar type
         */
        template <typename NewScalar>
        std::shared_ptr<Joint<NewScalar>> cast() const {
            std::shared_ptr<Joint<NewScalar>> new_joint = std::make_shared<Joint<NewScalar>>();
            new_joint->id = id;
            new_joint->q_index = q_index;
            new_joint->name = name;
            new_joint->type = type;
            new_joint->axis = axis.template cast<NewScalar>();
            new_joint->parent_transform = parent_transform.template cast<NewScalar>();
            new_joint->parent_link_name = parent_link_name;
            new_joint->child_transform = child_transform.template cast<NewScalar>();
            new_joint->child_link_name = child_link_name;
            return new_joint;
        }

    };
}  // namespace RML

#endif
