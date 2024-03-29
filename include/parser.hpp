#ifndef TR_URDFPARSER_HPP
#define TR_URDFPARSER_HPP

#include <tinyxml2.h>

#include "math.hpp"
#include "model.hpp"

/** \file parser.hpp
 * @brief Contains various functions for parsing URDF files and strings to create a tinyrobotics model.
 */
namespace tinyrobotics {

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
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rpy = vec_from_string<Scalar>(rotation_str);
        Eigen::Matrix<Scalar, 3, 3> R =
            Eigen::AngleAxis<Scalar>(rpy.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ())
            * Eigen::AngleAxis<Scalar>(rpy.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY())
            * Eigen::AngleAxis<Scalar>(rpy.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX()).toRotationMatrix();
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
                R                               = rpy_to_spatial(rpy);
            }

            const char* xyz_str = xml->Attribute("xyz");
            if (xyz_str != nullptr) {
                Eigen::Matrix<Scalar, 3, 1> translation = vec_from_string<Scalar>(xyz_str);
                T                                       = translation_to_spatial(translation);
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
            throw std::runtime_error("Error! Link without a name attribute detected!");
        }

        tinyxml2::XMLElement* i = xml->FirstChildElement("inertial");
        if (i != nullptr) {
            // Add the center of mass to the link
            tinyxml2::XMLElement* o = i->FirstChildElement("origin");
            if (o != nullptr) {
                link.center_of_mass = transform_from_xml<Scalar>(o);
            }

            // Add the mass to the link
            tinyxml2::XMLElement* mass_xml = i->FirstChildElement("mass");
            if (mass_xml != nullptr) {
                if (mass_xml->Attribute("value") != nullptr) {
                    try {
                        link.mass = std::stod(mass_xml->Attribute("value"));
                    }
                    catch (std::invalid_argument& e) {
                        throw std::runtime_error("Error while parsing link '" + std::string(get_parent_link_name(i))
                                                 + "': inertial mass [" + mass_xml->Attribute("value")
                                                 + "] is not a valid double: " + e.what() + "!");
                    }
                }
                else {
                    throw std::runtime_error("Error while parsing link '" + std::string(get_parent_link_name(i))
                                             + "' <mass> element must have a value attribute!");
                }
            }
            else {
                throw std::runtime_error("Error while parsing link '" + std::string(get_parent_link_name(i))
                                         + "' inertial element must have a <mass> element!");
            }
        }

        // Add the inertia to the link
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
                throw std::runtime_error("Error while parsing link '" + std::string(get_parent_link_name(i))
                                         + "' <inertia> element must have ixx,ixy,ixz,iyy,iyz,izz attributes!");
            }
        }
        else {
            throw std::runtime_error("Error while parsing link '" + std::string(get_parent_link_name(i))
                                     + "' inertial element must have a <inertia> element!");
        }

        // Add the spatial inertia to the link
        link.I = inertia_to_spatial<Scalar>(link.mass, link.center_of_mass.translation(), link.inertia);

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
                joint.axis << 1, 0, 0;
                joint.S << 1, 0, 0, 0, 0, 0;
            }
            else {
                const char* xyz_char = axis_xml->Attribute("xyz");
                if (xyz_char != nullptr) {
                    joint.axis = vec_from_string<Scalar>(std::string(xyz_char));
                }
            }

            if (joint.type == JointType::REVOLUTE) {
                joint.S << joint.axis, 0, 0, 0;
            }
            else if (joint.type == JointType::PRISMATIC) {
                joint.S << 0, 0, 0, joint.axis;
            }
        }
        // TODO: Add additional joint properties
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
     * @brief Initialize the link tree in the model.
     * @param model Tinyrobtics model.
     * @tparam Scalar Scalar type of the model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    void init_link_tree(Model<Scalar, nq>& model, std::vector<Joint<Scalar>>& joints) {
        // Initialize the joint count to zero
        model.n_q = 0;

        // Iterate over each joint in the model
        for (auto joint : joints) {
            // Check that the joint has a parent link and a child link specified
            std::string parent_link_name = joint.parent_link_name;
            if (parent_link_name.empty()) {
                throw std::runtime_error("Error while constructing model! Joint [" + joint.name
                                         + "] is missing a parent link specification.");
            }
            std::string child_link_name = joint.child_link_name;
            if (child_link_name.empty()) {
                throw std::runtime_error("Error while constructing model! Joint [" + joint.name
                                         + "] is missing a child link specification.");
            }

            // Get references to the child and parent links associated with the joint
            auto child_link = model.get_link(child_link_name);
            if (child_link.idx == -1) {
                throw std::runtime_error("Error while constructing model! Child link [" + child_link_name
                                         + "] of joint [" + joint.name + "] not found");
            }

            auto parent_link = model.get_link(parent_link_name);
            if (parent_link.idx == -1) {
                throw std::runtime_error("Error while constructing model! Parent link [" + parent_link_name
                                         + "] of joint [" + joint.name + "] not found");
            }

            child_link.parent = parent_link.idx;
            parent_link.add_child_link_idx(child_link.idx);

            // Only if the joint is of type REVOLUTE or PRISMATIC, add its index in the configuration vector
            if (joint.type == JointType::REVOLUTE || joint.type == JointType::PRISMATIC) {
                joint.idx = model.n_q;
                model.n_q++;
            }

            // Associate the joint with the child link and update the child link in the links vector
            child_link.joint            = joint;
            model.links[child_link.idx] = child_link;

            // Update the parent link in the links vector
            model.links[parent_link.idx] = parent_link;
        }

        // Find the base link of the model by finding the link with no parent link
        for (auto link : model.links) {
            bool found = false;
            if (link.parent == -1) {
                if (found) {
                    throw std::runtime_error(
                        "Error! Multiple base links found. The urdf does not contain a valid link tree.");
                }
                model.base_link_idx = link.idx;
                found               = true;
            }
        }
    }

    /**
     * @brief Updates dynamic links with any fixed joints associated with them, and updates the indices of the
     * dynamic links and their parents.
     * @param model Tinyrobtics model.
     * @tparam Scalar Scalar type of the model.
     * @tparam nq Number of configuration coordinates (degrees of freedom).
     */
    template <typename Scalar, int nq>
    void init_dynamics(Model<Scalar, nq>& model) {
        for (auto& link : model.links) {
            // If the link has a fixed joint, update the transforms of the child links and its parents link inertia
            if (link.joint.type == JointType::FIXED && link.idx != model.base_link_idx) {
                // Update fixed transforms of the child links
                for (auto child_link_idx : link.child_links) {
                    auto child_link = model.links[child_link_idx];
                    for (int j = 0; j < model.links.size(); j++) {
                        if (model.links[j].name == child_link.name) {
                            model.links[j].joint.X = model.links[j].joint.X * link.joint.X;
                            break;
                        }
                    }
                }
                // Combine spatial inertias
                auto parent_link = model.links[link.parent];
                // Add the spatial inertia of the link to its parent link in the link tree
                for (int j = 0; j < model.links.size(); j++) {
                    if (model.links[j].name == parent_link.name) {
                        Eigen::Matrix<Scalar, 6, 6> X_T = model.links[j].joint.X;
                        model.links[j].I += X_T.transpose() * link.I * X_T;
                        break;
                    }
                }
            }
            else if (link.idx != model.base_link_idx) {
                // Add the index of this actuatable link to the list of dynamic link indices
                model.q_map.push_back(link.idx);
            }
        }
        // For each dynamic link, find the index of its parent link in the dynamic link indices
        for (int i = 0; i < model.q_map.size(); i++) {
            // Get the dynamic link
            auto link = model.links[model.q_map[i]];
            // Get the parent link which isn't fixed
            auto parent_link = model.links[link.parent];
            while (parent_link.joint.type == JointType::FIXED) {
                parent_link = model.links[parent_link.parent];
            }
            // First check to see if the parent link is the base link
            if (parent_link.idx == model.base_link_idx) {
                // Add the index of the base link to the list of parent indices
                model.parent.push_back(-1);
            }
            else {
                // Find the parent link in the q_map
                bool found = false;
                for (int j = 0; j < model.q_map.size(); j++) {
                    if (model.q_map[j] == parent_link.idx) {
                        model.parent.push_back(j);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    model.parent.push_back(-1);
                }
            }
        }

        // Assign spatial gravity vector in models data
        model.spatial_gravity.tail(3) = model.gravity;

        // Compute the total mass of the model
        model.mass = 0;
        for (auto link : model.links) {
            model.mass += link.mass;
        }
    }

    /**
     * @brief Construct a new Model object from URDF file description.
     * @param xml_string The XML string of the URDF file.
     * @return The URDF parsed Model object.
     */
    template <typename Scalar, int nq>
    Model<Scalar, nq> import_urdf(const std::string& path_to_urdf) {
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

        // Set the models name
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
        std::vector<Joint<Scalar>> joints;
        for (tinyxml2::XMLElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml;
             joint_xml                       = joint_xml->NextSiblingElement("joint")) {
            Joint<Scalar> joint = joint_from_xml<Scalar>(joint_xml);
            for (auto& joint_ : joints) {
                if (joint.name == joint_.name) {
                    throw std::runtime_error("Error: Duplicate joints '" + joint.name + "' found");
                }
            }
            joints.push_back(joint);
        }

        // Initialize the link tree and find the base link index (should be -1)
        init_link_tree(model, joints);

        // Initialize the q_map and parent_map
        init_dynamics(model);

        return model;
    }
}  // namespace tinyrobotics

#endif