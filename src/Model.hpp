#ifndef RML_MODEL_HPP
#define RML_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <map>
#include <iomanip>

#include "Joint.hpp"
#include "Link.hpp"
#include "txml.h"

namespace RML {

    /**
     * @brief Represents a robot model.
     * @details
     * @param Scalar The scalar type for the robot model
     */
    template <typename Scalar>
    class Model {

        public:

        /// @brief The name of the robot model.
        std::string name = "";

        /// @brief The number of links in the robot model.
        int n_links = 0;

        /// @brief The number of joints in the robot model.
        int n_joints = 0;

        /// @brief The number of actuatable joints in the robot model.
        int n_q = 0;

        /// @brief The base link of the robot model.
        std::shared_ptr<Link<Scalar>> base_link = nullptr;

        /// @brief Map of links in the robot model.
        std::map<std::string, std::shared_ptr<Link<Scalar>>> links;

        /// @brief Map of joints in the robot model.
        std::map<std::string, std::shared_ptr<Joint<Scalar>>> joints;

        /// @brief The gravitational acceleration experienced by robot.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /**
         * @brief Construct a new Model object from URDF file description.
         * @param xml_string The XML string of the URDF file.
         */
        static std::shared_ptr<Model<Scalar>> from_urdf(const std::string& path_to_urdf) {

            std::shared_ptr<Model<Scalar>> model = std::make_shared<Model<Scalar>>();
            // Parse the URDF file into a string
            std::ifstream input_file(path_to_urdf);
            if (!input_file.is_open()) {
                std::cerr << "Could not open the file - '" << path_to_urdf << "'" << std::endl;
                exit(EXIT_FAILURE);
            }

            std::string urdf_string =
                std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());

            TiXmlDocument xml_doc;
            xml_doc.Parse(urdf_string.c_str());

            if (xml_doc.Error()) {
                std::string error_msg = xml_doc.ErrorDesc();
                xml_doc.ClearError();
                throw std::runtime_error(error_msg);
            }

            TiXmlElement *robot_xml = xml_doc.RootElement();
            if (robot_xml == nullptr || robot_xml->ValueStr() != "robot") {
                std::string error_msg = "Error! Could not find the <robot> element in the xml file";
                throw std::runtime_error(error_msg);
            }

            // ************************ Add the name to the model ************************
            const char *name = robot_xml->Attribute("name");
            if (name != nullptr){
                model->name = std::string(name);
            } else {
                std::string error_msg = "No name given for the robot. Please add a name attribute to the robot element!";
                throw std::runtime_error(error_msg);
            }

            // ************************ Add the links to the model ************************
            for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml != nullptr; link_xml = link_xml->NextSiblingElement("link")) {
            auto link = Link<Scalar>::fromXml(link_xml);

                if (model->get_link(link->name) != nullptr) {
                    std::ostringstream error_msg;
                    error_msg << "Error! Duplicate links '" << link->name << "' found!";
                    throw std::runtime_error(error_msg.str());
                } else {
                    model->links[link->name] = link;
                    model->n_links++;
                }
            }

            if (model->links.size() == 0){
                std::string error_msg = "Error! No link elements found in the urdf file.";
                throw std::runtime_error(error_msg);
            }

            // ************************ Add the joints to the model ************************
            for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml != nullptr; joint_xml = joint_xml->NextSiblingElement("joint")) {
                auto joint = Joint<Scalar>::fromXml(joint_xml);

                if (model->get_joint(joint->name) != nullptr) {
                    std::ostringstream error_msg;
                    error_msg << "Error! Duplicate joints '" << joint->name << "' found!";
                    throw std::runtime_error(error_msg.str());
                } else {
                    model->joints[joint->name] = joint;
                    model->n_joints++;
                }
            }

            std::map<std::string, std::string> parent_link_tree;
            model->init_link_tree(parent_link_tree);
            model->find_base(parent_link_tree);

            return model;
        }

        /**
         * @brief Initialize the link tree of the robot model.
         *
         */
        void init_link_tree(map<string, string>& parent_link_tree) {
            // Set count to zero
            int joint_q_index = 0;
            n_q = 0;

            for (auto joint = joints.begin(); joint != joints.end(); joint++) {

                string parent_link_name = joint->second->parent_link_name;
                string child_link_name = joint->second->child_link_name;

                if (parent_link_name.empty()){
                    ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint->first
                            << "] is missing a parent link specification.";
                    throw std::runtime_error(error_msg.str());
                }
                if (child_link_name.empty()) {
                    ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint->first
                            << "] is missing a child link specification.";
                    throw std::runtime_error(error_msg.str());
                }

                auto child_link = get_link(child_link_name);
                if (child_link == nullptr) {
                    ostringstream error_msg;
                    error_msg << "Error while constructing model! Child link [" << child_link_name
                            << "] of joint [" <<  joint->first << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                auto parent_link = get_link(parent_link_name);
                if (parent_link == nullptr) {
                    ostringstream error_msg;
                    error_msg << "Error while constructing model! Parent link [" << parent_link_name
                            << "] of joint [" <<  joint->first << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                child_link->parent_link = parent_link;
                child_link->joint = joint->second;

                parent_link->add_child_joint(joint->second);
                parent_link->add_child_link(child_link);

                parent_link_tree[child_link->name] = parent_link_name;

                // If the joint is actuatable, add its configuration vector q_index
                if(joint->second->type == JointType::REVOLUTE || joint->second->type == JointType::PRISMATIC) {
                    joint->second->q_index = joint_q_index;
                    joint_q_index++;
                    n_q++;
                }
            }

            // Assign IDs to the links and joints
            int id = 0;
            for (auto link = links.begin(); link != links.end(); link++) {
                if(link->second->joint != nullptr) {
                    link->second->joint->id = id;
                }
                link->second->id = id;
                id++;
            }
        }

        /**
         * @brief Find the base link of the robot model.
         *
         */
        void find_base(const map<string, string> &parent_link_tree) {
            for (auto l=links.begin(); l!=links.end(); l++) {
                auto parent = parent_link_tree.find(l->first);
                if (parent == parent_link_tree.end()) {
                    if (base_link == nullptr) {
                        base_link = get_link(l->first);
                    } else {
                        ostringstream error_msg;
                        error_msg << "Error! Multiple base links found: (" << base_link->name
                                << ") and (" + l->first + ")!";
                        throw std::runtime_error(error_msg.str());
                    }
                }
            }
            if (base_link == nullptr) {
                throw std::runtime_error("Error! No base link found. The urdf does not contain a valid link tree.");
            }
        }

        /**
         * @brief Get all the links in the robot model.
         *
         */
        const std::vector<std::shared_ptr<Link<Scalar>>> get_links() const {
            std::vector<std::shared_ptr<Link<Scalar>>> linklist;
            for (auto link = links.begin(); link != links.end(); link++) {
                linklist.push_back(link->second);
            }
            return linklist;
        }

        /**
         * @brief Get a link in the robot model.
         *
         */
        std::shared_ptr<Link<Scalar>> get_link(const std::string& name) {
            if (links.find(name) == links.end()) {
                return nullptr;
            } else {
                return links.find(name)->second;
            }
        }

        /**
         * @brief Get all the joints in the robot model.
         *
         */
        const std::vector<std::shared_ptr<Joint<Scalar>>> get_joints() const {
            std::vector<std::shared_ptr<Joint<Scalar>>> jointlist;
            for (auto joint = joints.begin(); joint != joints.end(); joint++) {
                jointlist.push_back(joint->second);
            }
            return jointlist;
        }

        /**
         * @brief Get a joint in the robot model.
         *
         */
        std::shared_ptr<Joint<Scalar>> get_joint(const std::string& name) {
            if (joints.find(name) == joints.end()) {
                return nullptr;
            } else {
                return joints.find(name)->second;
            }
        }

        /**
         * @brief Display details of the robot model.
         *
         */
        void show_details() {
            int spacing = 25;
            cout << "| ************************************************ Robot Model Details ************************************************ |" << endl;
            std::cout << "Name : " << name << std::endl;
            std::cout << "No. of links : " << n_links << std::endl;
            std::cout << "No. of joints : " << n_joints << std::endl;
            std::cout << "No. of actuatable joints : " << n_q << std::endl;
            std::cout << "Base link name : " << base_link->name << std::endl;
            std::cout << "Index" << std::setw(spacing) <<
            "Link Name" << std::setw(spacing) <<
            "Joint Name" << std::setw(15) <<
            "Joint Index" << std::setw(15) <<
            "Joint Type" << std::setw(spacing) <<
            "Parent Name" << std::setw(spacing) <<
            "Children Names" << std::endl;
            for (auto& link : this->get_links()) {

                std::string children_names = "";
                for (auto& child_link : link->child_joints) {
                    children_names += child_link->name + " ";
                }

                if(link->joint != nullptr) {
                    std::cout << link->id << std::setw(spacing) <<
                    link->name << std::setw(spacing) <<
                    link->joint->name << std::setw(15) <<
                    link->joint->q_index << std::setw(15) <<
                    int(link->joint->type) <<  std::setw(spacing) <<
                    link->parent_link->name <<  std::setw(spacing+15) <<
                    children_names <<  std::endl << std::endl;
                }
            }
            return;
        }

        /**
         * @brief Get a configuration vector for the robot model of all zeros.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> home_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q(n_q);
            q.setZero();
            return q;
        }

        /**
         * @brief Get a random configuration vector for the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> random_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q(n_q);
            q.setRandom();
            q =  M_PI*q;
            return q;
        }

        /**
         * @brief Cast to NewScalar type.
         *
         */
        template<typename NewScalar>
        std::shared_ptr<Model<NewScalar>> cast() {
            std::shared_ptr<Model<NewScalar>> new_model;
            new_model = std::make_shared<Model<NewScalar>>();
            new_model->name = name;
            new_model->n_links = n_links;
            new_model->n_joints = n_joints;
            new_model->n_q = n_q;
            new_model->base_link = base_link->template cast<NewScalar>();
            new_model->gravity = gravity.template cast<NewScalar>();
            for (auto& link : links) {
                new_model->links[link.first] = link.second->template cast<NewScalar>();
            }
            for (auto& joint : joints) {
                new_model->joints[joint.first] = joint.second->template cast<NewScalar>();
            }
            std::map<std::string, std::string> parent_link_tree;
            new_model->init_link_tree(parent_link_tree);
            // new_model->find_base(parent_link_tree);
            return new_model;
        }
    };
}  //namespace RML

#endif
