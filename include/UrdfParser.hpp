#ifndef RML_URDFPARSER_HPP
#define RML_URDFPARSER_HPP

#include <tinyxml2.h>

#include "Math.hpp"
#include "Model.hpp"

namespace RML {

    /**
     * @brief Get Eigen3 vector from a URDF vector element.
     * @param vector_str The vector string
     * @return A 3x1 Eigen3 vector
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 1> vec_from_string(const std::string& vector_str) {
        Eigen::Matrix<Scalar, 3, 1> vec;
        std::vector<Scalar> values;
        std::istringstream ss(vector_str);
        std::string vector_element;
        while (ss >> vector_element) {
            try {
                values.push_back(std::stod(vector_element));
            }
            catch (std::invalid_argument& e) {
                throw std::runtime_error("Error not able to parse component (" + vector_element
                                         + ") to a Scalar (while parsing a vector value)");
            }
        }

        if (values.size() != 3) {
            std::ostringstream error_msg;
            error_msg << "Parser found " << values.size() << " elements but 3 expected while parsing vector ["
                      << vector_str << "]";
            throw std::runtime_error(error_msg.str());
        }

        vec.x() = values[0];
        vec.y() = values[1];
        vec.z() = values[2];

        return vec;
    }

    /**
     * @brief Get Eigen3 rotation from a URDF rpy string
     * @param rotation_str The rpy string
     * @return A 3x3 Eigen3 rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rot_from_string(const std::string& rotation_str) {
        Eigen::Matrix<Scalar, 3, 1> rpy = vec_from_string<Scalar>(rotation_str);
        Eigen::Matrix<Scalar, 3, 3> R;
        R = Eigen::AngleAxis<Scalar>(rpy.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX())
            * Eigen::AngleAxis<Scalar>(rpy.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY())
            * Eigen::AngleAxis<Scalar>(rpy.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        return R;
    }

    /**
     * @brief Get the Eigen3 transform from a URDF xml element.
     * @param xml The XML element
     * @return A Eigen3 affine transform
     */
    template <typename Scalar>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> transform_from_xml(tinyxml2::XMLElement* xml) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> t;
        t.setIdentity();
        if (xml) {
            const char* rpy_str = xml->Attribute("rpy");
            if (rpy_str != nullptr) {
                t.linear() = rot_from_string<Scalar>(rpy_str);
            }

            const char* xyz_str = xml->Attribute("xyz");
            if (xyz_str != nullptr) {
                t.translation() = vec_from_string<Scalar>(xyz_str);
            }
        }
        return t;
    }

    /**
     * @brief Get the spatial transform from a URDF xml element.
     * @param xml The XML element
     * @return A spatial transform
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> spatial_transform_from_xml(tinyxml2::XMLElement* xml) {
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();
        Eigen::Matrix<Scalar, 6, 6> R = Eigen::Matrix<Scalar, 6, 6>::Identity();
        Eigen::Matrix<Scalar, 6, 6> T = Eigen::Matrix<Scalar, 6, 6>::Identity();
        if (xml) {
            const char* rpy_str = xml->Attribute("rpy");
            if (rpy_str != nullptr) {
                Eigen::Matrix<Scalar, 3, 1> rpy = vec_from_string<Scalar>(rpy_str);
                R                               = RML::R(rpy);
            }

            const char* xyz_str = xml->Attribute("xyz");
            if (xyz_str != nullptr) {
                Eigen::Matrix<Scalar, 3, 1> translation = vec_from_string<Scalar>(xyz_str);
                T                                       = RML::xlt(translation);
            }
        }
        X = R * T;
        return X;
    }

    /**
     * @brief Get the name of the link's parent link.
     * @param xml The XML element containing the link description
     * @return The name of the link's parent link.
     */
    inline const char* get_parent_link_name(tinyxml2::XMLElement* c) {
        tinyxml2::XMLElement* e = c->Parent()->ToElement();
        while (e->Parent() != nullptr) {
            if (std::strcmp(e->Value(), "link") == 0) {
                break;
            }
            e = e->Parent()->ToElement();
        }
        return e->Attribute("name");
    }

    /**
     * @brief Construct a Link from a URDF xml element.
     * @param xml The XML element containing the link description
     * @return The Link object
     */
    template <typename Scalar>
    Link<Scalar> link_from_xml(tinyxml2::XMLElement* xml) {
        // Create the link object
        Link<Scalar> link = Link<Scalar>();

        const char* name_char = xml->Attribute("name");
        if (name_char != nullptr) {
            link.name = std::string(name_char);
        }
        else {
            std::ostringstream error_msg;
            error_msg << "Error! Link without a name attribute detected!";
            throw std::runtime_error(error_msg.str());
        }

        tinyxml2::XMLElement* i = xml->FirstChildElement("inertial");
        if (i != nullptr) {
            // ************************ Add the centre of mass to the link ************************
            tinyxml2::XMLElement* o = i->FirstChildElement("origin");
            if (o != nullptr) {
                link.centre_of_mass = transform_from_xml<Scalar>(o);
            }
            // ************************ Add the mass to the link ************************
            tinyxml2::XMLElement* mass_xml = i->FirstChildElement("mass");
            if (mass_xml != nullptr) {
                if (mass_xml->Attribute("value") != nullptr) {
                    try {
                        link.mass = std::stod(mass_xml->Attribute("value"));
                    }
                    catch (std::invalid_argument& e) {
                        std::ostringstream error_msg;
                        error_msg << "Error while parsing link '" << get_parent_link_name(i) << "': inertial mass ["
                                  << mass_xml->Attribute("value") << "] is not a valid double: " << e.what() << "!";
                        throw std::runtime_error(error_msg.str());
                    }
                }
                else {
                    std::ostringstream error_msg;
                    error_msg << "Error while parsing link '" << get_parent_link_name(i)
                              << "' <mass> element must have a value attribute!";
                    throw std::runtime_error(error_msg.str());
                }
            }
            else {
                std::ostringstream error_msg;
                error_msg << "Error while parsing link '" << get_parent_link_name(i)
                          << "' inertial element must have a <mass> element!";
                throw std::runtime_error(error_msg.str());
            }
        }
        // ************************ Add the inertia to the link ************************
        tinyxml2::XMLElement* inertia_xml = i->FirstChildElement("inertia");
        if (inertia_xml != nullptr) {
            Eigen::Matrix<Scalar, 3, 3> inertia;
            if (inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz")
                && inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") && inertia_xml->Attribute("izz")) {
                try {
                    inertia(0, 0) = std::stod(inertia_xml->Attribute("ixx"));
                    inertia(0, 1) = std::stod(inertia_xml->Attribute("ixy"));
                    inertia(0, 2) = std::stod(inertia_xml->Attribute("ixz"));
                    inertia(1, 0) = std::stod(inertia_xml->Attribute("ixy"));
                    inertia(1, 1) = std::stod(inertia_xml->Attribute("iyy"));
                    inertia(1, 2) = std::stod(inertia_xml->Attribute("iyz"));
                    inertia(2, 0) = std::stod(inertia_xml->Attribute("ixz"));
                    inertia(2, 1) = std::stod(inertia_xml->Attribute("iyz"));
                    inertia(2, 2) = std::stod(inertia_xml->Attribute("izz"));
                    link.inertia  = inertia;
                }
                catch (std::invalid_argument& e) {
                    std::ostringstream error_msg;
                    error_msg << "Error while parsing link '" << get_parent_link_name(i)
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
                error_msg << "Error while parsing link '" << get_parent_link_name(i)
                          << "' <inertia> element must have ixx,ixy,ixz,iyy,iyz,izz attributes!";
                throw std::runtime_error(error_msg.str());
            }
        }
        else {
            std::ostringstream error_msg;
            error_msg << "Error while parsing link '" << get_parent_link_name(i)
                      << "' inertial element must have a <inertia> element!";
            throw std::runtime_error(error_msg.str());
        }

        // Add the spatial inertia to the link
        link.I = RML::spatial_inertia<Scalar>(link.mass, link.centre_of_mass.translation(), link.inertia);

        return link;
    }

    /**
     * @brief Construct a new Joint object from a URDF file description
     * @param xml The XML element containing the joint description
     * @return The Joint object
     */
    template <typename Scalar>
    Joint<Scalar> joint_from_xml(tinyxml2::XMLElement* xml) {
        Joint<Scalar> joint = Joint<Scalar>();

        const char* name = xml->Attribute("name");
        if (name != nullptr) {
            joint.name = std::string(name);
        }
        else {
            std::ostringstream error_msg;
            error_msg << "Error while parsing model: unnamed joint found!";
            throw std::runtime_error(error_msg.str());
        }

        tinyxml2::XMLElement* origin_xml = xml->FirstChildElement("origin");
        if (origin_xml != nullptr) {
            joint.parent_transform = transform_from_xml<Scalar>(origin_xml);
            joint.X                = spatial_transform_from_xml<Scalar>(origin_xml);
        }

        tinyxml2::XMLElement* parent_xml = xml->FirstChildElement("parent");
        if (parent_xml != nullptr) {
            const char* pname = parent_xml->Attribute("link");
            if (pname != nullptr) {
                joint.parent_link_name = std::string(pname);
            }
            // if no parent link name specified. this might be the root node
        }

        tinyxml2::XMLElement* child_xml = xml->FirstChildElement("child");
        if (child_xml) {
            const char* pname = child_xml->Attribute("link");
            if (pname != nullptr) {
                joint.child_link_name = std::string(pname);
            }
        }

        const char* type_char = xml->Attribute("type");
        if (type_char == nullptr) {
            std::ostringstream error_msg;
            error_msg << "Error! Joint " << joint.name << " has no type, check to see if it's a reference.";
            throw std::runtime_error(error_msg.str());
        }

        std::string type_str = type_char;
        if (type_str == "planar") {
            joint.type = JointType::PLANAR;
        }
        else if (type_str == "floating") {
            joint.type = JointType::FLOATING;
        }
        else if (type_str == "revolute") {
            joint.type = JointType::REVOLUTE;
        }
        else if (type_str == "continuous") {
            joint.type = JointType::CONTINUOUS;
        }
        else if (type_str == "prismatic") {
            joint.type = JointType::PRISMATIC;
        }
        else if (type_str == "fixed") {
            joint.type = JointType::FIXED;
        }
        else {
            std::ostringstream error_msg;
            error_msg << "Error! Joint '" << joint.name << "' has unknown type (" << type_str << ")!";
            throw std::runtime_error(error_msg.str());
        }

        if (joint.type != JointType::FLOATING && joint.type != JointType::FIXED) {
            tinyxml2::XMLElement* axis_xml = xml->FirstChildElement("axis");
            if (axis_xml == nullptr) {
                Eigen::Matrix<Scalar, 3, 1> default_axis;
                default_axis << 1, 0, 0;
                joint.axis = default_axis;
            }
            else {
                const char* xyz_char = axis_xml->Attribute("xyz");
                if (xyz_char != nullptr) {
                    joint.axis = vec_from_string<Scalar>(std::string(xyz_char));
                }
            }
        }

        // Add spatial transform to the Xtree
        // joint.X = RML::xlt(joint.parent_transform.inverse().matrix());

        // tinyxml2::XMLElement *prop_xml = xml->FirstChildElement("dynamics");
        // if (prop_xml != nullptr) {
        //     joint.dynamics = JointDynamics::fromXml(prop_xml);
        // }

        // tinyxml2::XMLElement *limit_xml = xml->FirstChildElement("limit");
        // if (limit_xml != nullptr) {
        //     joint.limits = JointLimits::fromXml(limit_xml);
        // }

        // tinyxml2::XMLElement *safety_xml = xml->FirstChildElement("safety_controller");
        // if (safety_xml != nullptr) {
        //     joint.safety = JointSafety::fromXml(safety_xml);
        // }

        // tinyxml2::XMLElement *calibration_xml = xml->FirstChildElement("calibration");
        // if (calibration_xml != nullptr) {
        //     joint.calibration = JointCalibration::fromXml(calibration_xml);
        // }

        // tinyxml2::XMLElement *mimic_xml = xml->FirstChildElement("mimic");
        // if (mimic_xml != nullptr) {
        //     joint.mimic = JointMimic::fromXml(mimic_xml);
        // }

        return joint;
    }

    /**
     * @brief Construct a new Model object from URDF file description.
     * @param xml_string The XML string of the URDF file.
     * @return The URDF parsed Model object.
     */
    template <typename Scalar, int nq>
    Model<Scalar, nq> model_from_urdf(const std::string& path_to_urdf) {
        // Open the URDF file
        std::ifstream input_file(path_to_urdf);
        if (!input_file.is_open()) {
            throw std::runtime_error("Could not open the file - '" + path_to_urdf + "'");
        }

        // Read the URDF file into an XML string
        std::string urdf_string =
            std::string(std::istreambuf_iterator<char>(input_file), std::istreambuf_iterator<char>());

        // Parse the XML string using tinyxml2
        tinyxml2::XMLDocument xml_doc;
        xml_doc.Parse(urdf_string.c_str());
        if (xml_doc.Error()) {
            throw std::runtime_error("Error parsing XML: " + std::string(xml_doc.ErrorStr()));
        }

        // Get the robot element
        tinyxml2::XMLElement* robot_xml = xml_doc.RootElement();
        if (!robot_xml || std::string(robot_xml->Value()) != "robot") {
            throw std::runtime_error("Error: Could not find the <robot> element in the URDF file");
        }

        // Create a new model
        Model<Scalar, nq> model;

        // Get the robot's name
        const char* name = robot_xml->Attribute("name");
        if (!name) {
            throw std::runtime_error("Error: No name given for the robot");
        }
        model.name = name;

        // Parse the links
        for (tinyxml2::XMLElement* link_xml = robot_xml->FirstChildElement("link"); link_xml;
             link_xml                       = link_xml->NextSiblingElement("link")) {
            Link<Scalar> link = link_from_xml<Scalar>(link_xml);
            if (model.get_link(link.name).idx != -1) {
                throw std::runtime_error("Error: Duplicate links '" + link.name + "' found");
            }
            link.idx = model.links.size();
            model.links.push_back(link);
        }
        if (model.links.empty()) {
            throw std::runtime_error("Error: No link elements found in the URDF file");
        }

        // Parse the joints
        for (tinyxml2::XMLElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml;
             joint_xml                       = joint_xml->NextSiblingElement("joint")) {
            Joint<Scalar> joint = joint_from_xml<Scalar>(joint_xml);
            if (model.get_joint(joint.name).joint_id != -1) {
                throw std::runtime_error("Error: Duplicate joints '" + joint.name + "' found");
            }
            joint.joint_id = model.joints.size();
            model.joints.push_back(joint);
        }
        model.n_joints = model.joints.size();
        model.n_links  = model.links.size();

        // Initialize the link tree and find the base link
        model.init_link_tree();
        // Initialize the q_idx and parent_map
        model.init_dynamics();

        // Resize the results structure
        model.data.resize(model.n_q);

        return model;
    }
}  // namespace RML

#endif