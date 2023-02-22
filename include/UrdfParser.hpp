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
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rpy = vec_from_string<Scalar>(rotation_str);
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
            const char* xyz_str = xml->Attribute("xyz");
            if (xyz_str != nullptr) {
                t.translation() = vec_from_string<Scalar>(xyz_str);
            }

            const char* rpy_str = xml->Attribute("rpy");
            if (rpy_str != nullptr) {
                t.linear() = rot_from_string<Scalar>(rpy_str);
            }
        }
        return t;
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
    static Model<Scalar, nq> model_from_urdf(const std::string& path_to_urdf) {

        Model<Scalar, nq> model = Model<Scalar, nq>();
        // Parse the URDF file into a string
        std::ifstream input_file(path_to_urdf);
        if (!input_file.is_open()) {
            std::cerr << "Could not open the file - '" << path_to_urdf << "'" << std::endl;
            exit(EXIT_FAILURE);
        }

        std::string urdf_string =
            std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());

        tinyxml2::XMLDocument xml_doc;
        xml_doc.Parse(urdf_string.c_str());

        if (xml_doc.Error()) {
            std::string error_msg = xml_doc.ErrorStr();
            xml_doc.ClearError();
            throw std::runtime_error(error_msg);
        }

        tinyxml2::XMLElement* robot_xml = xml_doc.RootElement();
        if (std::string(robot_xml->Value()) != "robot") {
            std::cout << "robot_xml->Value() = " << robot_xml->Value() << std::endl;
            std::string error_msg = "Error! Could not find the <robot> element in the xml file";
            throw std::runtime_error(error_msg);
        }

        // ************************ Add the name to the model ************************
        const char* name = robot_xml->Attribute("name");
        if (name != nullptr) {
            model.name = std::string(name);
        }
        else {
            std::string error_msg = "No name given for the robot. Please add a name attribute to the robot element!";
            throw std::runtime_error(error_msg);
        }

        // ************************ Add the links to the model ************************
        for (tinyxml2::XMLElement* link_xml = robot_xml->FirstChildElement("link"); link_xml != nullptr;
             link_xml                       = link_xml->NextSiblingElement("link")) {
            Link<Scalar> link = link_from_xml<Scalar>(link_xml);
            if (model.get_link(link.name).link_idx != -1) {
                std::ostringstream error_msg;
                error_msg << "Error! Duplicate links '" << link.name << "' found!";
                throw std::runtime_error(error_msg.str());
            }
            else {
                // Assign the link index
                link.link_idx = model.links.size();
                // Add the link to the model
                model.links.push_back(link);
            }
        }

        if (model.links.size() == 0) {
            std::string error_msg = "Error! No link elements found in the urdf file.";
            throw std::runtime_error(error_msg);
        }

        // ************************ Add the joints to the model ************************
        for (tinyxml2::XMLElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml != nullptr;
             joint_xml                       = joint_xml->NextSiblingElement("joint")) {
            auto joint = joint_from_xml<Scalar>(joint_xml);

            if (model.get_joint(joint.name).joint_idx != -1) {
                std::ostringstream error_msg;
                error_msg << "Error! Duplicate joints '" << joint.name << "' found!";
                throw std::runtime_error(error_msg.str());
            }
            else {
                // Assign the joint index
                joint.joint_idx = model.joints.size();
                model.joints.push_back(joint);
            }
        }

        std::map<std::string, std::string> parent_link_tree;
        model.init_link_tree(parent_link_tree);
        model.find_base(parent_link_tree);
        model.n_joints = model.joints.size();
        model.n_links  = model.links.size();

        // ************************ Add the dynamic links to the model ************************
        for (int i = 1; i < model.n_links; i++) {
            auto link_i  = model.links[i];
            auto joint_i = model.joints[link_i.joint_idx];
            if (joint_i.type == JointType::FIXED) {
                // Merge the spatial inertia of the link with its parent
                auto parent_link = model.links[link_i.parent_link_idx];
                Eigen::Matrix<Scalar, 6, 6> I =
                    RML::spatial_inertia<Scalar>(link_i.mass, link_i.centre_of_mass.translation(), link_i.inertia);
                // Add the spatial inertia of the link to the parent link in the dynamic link tree, TODO: We should
                // probably transform the inertia to the parent frame, for now just don't support fixed joints with an
                // offset

                for (int j = 0; j < model.dynamic_links.size(); j++) {
                    if (model.dynamic_links[j].link.name == parent_link.name) {
                        Eigen::Matrix<Scalar, 6, 6> X_T = model.dynamic_links[j].X;
                        Eigen::Matrix<Scalar, 6, 6> I_T = X_T.transpose() * I;  // * X_T;
                        model.dynamic_links[j].I += I_T;
                        break;
                    }
                }
            }
            else {
                // Create a new dynamic link
                DynamicLink<Scalar> dynamic_link = DynamicLink<Scalar>();
                // Add its associated joint
                dynamic_link.joint = joint_i;
                // Add its associated link
                dynamic_link.link = link_i;
                // Add its spatial transform
                Eigen::Matrix<Scalar, 6, 6> X_T = RML::xlt(joint_i.parent_transform.matrix());
                dynamic_link.X                  = X_T;
                // Add its spatial inertia
                Eigen::Matrix<Scalar, 6, 6> I =
                    RML::spatial_inertia<Scalar>(link_i.mass, link_i.centre_of_mass.translation(), link_i.inertia);
                dynamic_link.I = I;
                model.dynamic_links.push_back(dynamic_link);
            }
        }

        // For each dynamic link, find its closest parent link that is not fixed in the dynamic link tree and add it to
        // the dynamic link
        for (int i = 0; i < model.dynamic_links.size(); i++) {
            auto dynamic_link_i = model.dynamic_links[i];
            // Find the closest parent link that is not fixed
            auto dynamic_link_parent = model.links[dynamic_link_i.link.parent_link_idx];
            // Check if the parent is fixed
            if (model.joints[dynamic_link_parent.joint_idx].type == JointType::FIXED) {
                // If the parent is fixed, find the closest parent that is not fixed
                while (model.joints[dynamic_link_parent.joint_idx].type == JointType::FIXED) {
                    dynamic_link_parent = model.links[dynamic_link_parent.parent_link_idx];
                }
            }
            // Find the index of the closest parent link that is not fixed in the dynamic link tree
            int parent_link_idx = -1;
            for (int j = 0; j < model.dynamic_links.size(); j++) {
                if (model.dynamic_links[j].link.name == dynamic_link_parent.name) {
                    parent_link_idx = j;
                    break;
                }
            }
            // Add the parent index to the dynamic link
            model.dynamic_links[i].parent_link_idx = parent_link_idx;
        }


        std::cout << "Xtree.size() = " << model.Xtree.size() << std::endl;
        std::cout << "I.size() = " << model.I.size() << std::endl;
        std::cout << "parent.size() = " << model.parent.size() << std::endl;

        // Starting from the base link, add the dynamic links (links with non-fixed joints) to the model, this includes
        // the spatial inertia and spatial transform and the parent index

        for (int i = 0; i < model.Xtree.size(); i++) {
            std::cout << "Xtree[" << i << "] = " << std::endl << model.Xtree[i] << std::endl;
            std::cout << "model.dynamic_links[" << i << "].X = " << std::endl << model.dynamic_links[i].X << std::endl;
            std::cout << "I[" << i << "] = " << std::endl << model.I[i] << std::endl;
            std::cout << "model.dynamic_links[" << i << "].I = " << std::endl << model.dynamic_links[i].I << std::endl;
            std::cout << "parent[" << i << "] = " << model.parent[i] << std::endl;
            std::cout << "model.dynamic_links[" << i << "].parent = " << model.dynamic_links[i].link.parent_link_idx
                      << std::endl;
        }

        // Resize the results structure with number of actuatable joints
        model.data.resize(model.n_q);

        return model;
    }
}  // namespace RML

#endif