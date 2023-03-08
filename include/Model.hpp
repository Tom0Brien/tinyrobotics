#ifndef RML_MODEL_HPP
#define RML_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

#include "Data.hpp"
#include "Joint.hpp"
#include "Link.hpp"

namespace RML {

    /**
     * @brief A robot model.
     * @details A robot model is a collection of links and joints that can be used to represent a robot.
     * @param Scalar The scalar type for the robot model
     */
    template <typename Scalar, int nq>
    struct Model {
        /// @brief The name of the robot model.
        std::string name = "";

        /// @brief The number of links in the robot model.
        int n_links = 0;

        /// @brief The number of joints in the robot model.
        int n_joints = 0;

        /// @brief The number of actuatable joints in the robot model.
        int n_q = 0;

        /// @brief The index of the base link in the robot model.
        int base_link_idx = -1;

        /// @brief Vector of links in the robot model.
        std::vector<Link<Scalar>> links = {};

        /// @brief Vector of dynamic links in the robot model (links with joints that can be actuated)
        std::vector<Link<Scalar>> dynamic_links = {};

        /// @brief Vector of joints in the robot model.
        std::vector<Joint<Scalar>> joints = {};

        /// @brief The gravitational acceleration experienced by robot.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Stores the results of the models algorithms.
        Data<Scalar, nq> data;

        /// @brief @brief Initialize the link tree of the robot model.
        void init_link_tree() {
            // Initialize the joint count to zero
            n_q = 0;

            // Iterate over each joint in the robot model
            for (auto joint : joints) {
                // Check that the joint has a parent link and a child link specified
                std::string parent_link_name = joint.parent_link_name;
                if (parent_link_name.empty()) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint.name
                              << "] is missing a parent link specification.";
                    throw std::runtime_error(error_msg.str());
                }
                std::string child_link_name = joint.child_link_name;
                if (child_link_name.empty()) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint.name
                              << "] is missing a child link specification.";
                    throw std::runtime_error(error_msg.str());
                }

                // Get references to the child and parent links associated with the joint
                auto child_link = get_link(child_link_name);
                if (child_link.link_idx == -1) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Child link [" << child_link_name << "] of joint ["
                              << joint.name << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                auto parent_link = get_link(parent_link_name);
                if (parent_link.link_idx == -1) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Parent link [" << parent_link_name << "] of joint ["
                              << joint.name << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                // Set the parent link index for the child link and add the child link index to the parent link's list
                // of child link indices
                child_link.parent = parent_link.link_idx;
                parent_link.add_child_link_idx(child_link.link_idx);

                // If the joint is of type REVOLUTE or PRISMATIC, add its index in the configuration vector and
                // increment the configuration vector size
                if (joint.type == JointType::REVOLUTE || joint.type == JointType::PRISMATIC) {
                    joint.q_idx = n_q;
                    n_q++;
                }

                // Associate the joint with the child link and update the child link in the links vector
                child_link.joint           = joint;
                links[child_link.link_idx] = child_link;

                // Update the parent link in the links vector
                links[parent_link.link_idx] = parent_link;
            }

            // Find the base link of the robot model by finding the link with no parent link
            for (auto link : links) {
                bool found = false;
                if (link.parent == -1) {
                    if (found) {
                        throw std::runtime_error(
                            "Error! Multiple base links found. The urdf does not contain a valid link tree.");
                    }
                    base_link_idx = link.link_idx;
                    found         = true;
                }
            }
        }

        /// @brief Initialize the dynamic link tree of the robot model.
        void init_dynamic_link_tree() {
            dynamic_links = links;
            // Remove the base link from the dynamic links
            dynamic_links.erase(dynamic_links.begin() + base_link_idx);
            for (int i = 0; i < dynamic_links.size(); i++) {
                auto link = dynamic_links[i];
                if (link.joint.type == JointType::FIXED) {
                    // Update fixed transforms
                    for (auto child_link_idx : link.child_links) {
                        auto child_link = links[child_link_idx];
                        for (int j = 0; j < dynamic_links.size(); j++) {
                            if (dynamic_links[j].name == child_link.name) {
                                Eigen::Matrix<Scalar, 6, 6> X_T = dynamic_links[j].joint.X;
                                dynamic_links[j].joint.X        = X_T * link.joint.X;
                                break;
                            }
                        }
                    }
                    // Combine spatial inertias
                    auto parent_link = links[link.parent];
                    // Add the spatial inertia of the link to its parent link in the dynamic link tree
                    for (int j = 0; j < dynamic_links.size(); j++) {
                        if (dynamic_links[j].name == parent_link.name) {
                            Eigen::Matrix<Scalar, 6, 6> X_T = dynamic_links[j].joint.X;
                            Eigen::Matrix<Scalar, 6, 6> I_T = X_T.transpose() * link.I * X_T;
                            dynamic_links[j].I += I_T;
                            break;
                        }
                    }
                    // Remove the fixed link from the dynamic link tree
                    dynamic_links.erase(dynamic_links.begin() + i);
                }
            }

            // Assign the link/parent link indices
            for (int i = 0; i < dynamic_links.size(); i++) {
                dynamic_links[i].link_idx = i;
                if (dynamic_links[i].parent == -1) {
                    dynamic_links[i].parent = -1;
                }
                else {
                    // Find the parent link in the full link tree which doesn't have a fixed joint
                    // and assign the parent link index
                    auto parent_link = links[dynamic_links[i].parent];
                    while (parent_link.joint.type == JointType::FIXED) {
                        parent_link = links[parent_link.parent];
                    }
                    // Find the parent links index in the dynamic link tree
                    for (int j = 0; j < dynamic_links.size(); j++) {
                        if (dynamic_links[j].name == parent_link.name) {
                            dynamic_links[i].parent = j;
                            break;
                        }
                    }
                }
            }
        }

        /**
         * @brief Get a link in the robot model.
         * @param name The name of the link.
         * @return The link
         */
        Link<Scalar> get_link(const std::string& name) const {
            for (auto link : links) {
                if (link.name == name) {
                    return link;
                }
            }
            // No link was found
            return Link<Scalar>();
        }

        /**
         * @brief Get the parent link in the robot model.
         * @param link_idx The index of the link in the robot model.
         * @return The parent link of the link
         *
         */
        Link<Scalar> get_parent_link(const std::string& name) const {
            for (auto link : links) {
                if (link.name == name) {
                    return links[link.parent];
                }
            }
            // No link was found
            throw std::runtime_error("Error! Parent of Link [" + name + "] not found!");
            return Link<Scalar>();
        }

        /**
         * @brief Get the joint in the robot model.
         * @param name The name of the joint.
         * @return The joint in the robot model.
         *
         */
        Joint<Scalar> get_joint(const std::string& name) const {
            for (auto joint : joints) {
                if (joint.name == name) {
                    return joints[joint.joint_idx];
                }
            }
            // No joint was found
            return Joint<Scalar>();
        }

        /**
         * @brief Display details of the robot model.
         *
         */
        void show_details() {
            int spacing = 25;
            std::cout << "| ************************************************ Robot Model Details "
                         "************************************************ |"
                      << std::endl;
            std::cout << "Name : " << name << std::endl;
            std::cout << "No. of links : " << links.size() << std::endl;
            std::cout << "No. of joints : " << joints.size() << std::endl;
            std::cout << "No. of actuatable joints : " << n_q << std::endl;
            std::cout << "Base link name : " << links[base_link_idx].name << std::endl;
            std::cout << "Index" << std::setw(spacing) << "Link Name" << std::setw(spacing) << "Joint Name"
                      << std::setw(15) << "Joint Index" << std::setw(15) << "Joint Type" << std::setw(spacing)
                      << "Parent Name" << std::setw(spacing) << "Children Names" << std::endl;
            for (auto& link : links) {
                std::string children_names = "";
                // for (auto& child_joint_idx : link.child_joints) {
                //     children_names += joints[child_joint_idx].child_link_name + " ";
                // }
                // std::cout << "link.joint_idx : " << link.joint_idx << std::endl;
                if (link.joint.joint_idx != -1) {
                    std::cout << link.link_idx << std::setw(spacing) << link.name << std::setw(spacing)
                              << link.joint.name << std::setw(15) << link.joint.q_idx << std::setw(15)
                              << int(link.joint.type) << std::setw(spacing + 15) << link.parent << std::setw(spacing)
                              << children_names << std::endl
                              << std::endl;
                }
            }
        }

        /**
         * @brief Get a configuration vector for the robot model of all zeros.
         *
         */
        Eigen::Matrix<Scalar, nq, 1> home_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, nq, 1> q(n_q);
            q.setZero();
            return q;
        }

        /**
         * @brief Get a random configuration vector for the robot model.
         *
         */
        Eigen::Matrix<Scalar, nq, 1> random_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, nq, 1> q(n_q);
            q.setRandom();
            q = M_PI * q;
            return q;
        }

        /**
         * @brief Cast to NewScalar type.
         *
         */
        template <typename NewScalar>
        Model<NewScalar, nq> cast() {
            Model<NewScalar, nq> new_model = Model<NewScalar, nq>();
            new_model.name                 = name;
            new_model.n_links              = n_links;
            new_model.n_joints             = n_joints;
            new_model.n_q                  = n_q;
            new_model.base_link_idx        = base_link_idx;
            new_model.gravity              = gravity.template cast<NewScalar>();
            for (auto& joint : joints) {
                new_model.joints.push_back(joint.template cast<NewScalar>());
            }
            for (auto& link : links) {
                new_model.links.push_back(link.template cast<NewScalar>());
            }
            new_model.data = data.template cast<NewScalar>();
            return new_model;
        }
    };
}  // namespace RML

#endif