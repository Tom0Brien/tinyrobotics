#ifndef TR_MODEL_HPP
#define TR_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>

#include "Data.hpp"
#include "Joint.hpp"
#include "Link.hpp"

namespace tr {

    /**
     * @brief A tinyrobotics model.
     * @details A tinyrobotics model is a collection of links and joints that represents a robot.
     * @tparam Scalar The scalar type for the model.
     * @tparam nq The number of configuration coordinates (number of degrees of freedom).
     */
    template <typename Scalar, int nq>
    struct Model {
        /// @brief Name of the model.
        std::string name = "";

        /// @brief Number of links in the model.
        int n_links = 0;

        /// @brief Number of joints in the model.
        int n_joints = 0;

        /// @brief Number of configuration coordinates (degrees of freedom) in the model.
        int n_q = 0;

        /// @brief Vector of links in the model.
        std::vector<Link<Scalar>> links = {};

        /// @brief Index of the base link in the models links vector.
        int base_link_idx = -1;

        /// @brief Vector of indices of dynamic links in the models link vector (links with joints that can be
        /// actuated).
        std::vector<int> q_idx = {};

        /// @brief Vector of parent link indices of links which are not fixed (links with joints that can be actuated).
        std::vector<int> parent = {};

        /// @brief Vector of joints in the model.
        std::vector<Joint<Scalar>> joints = {};

        /// @brief Gravitational acceleration experienced by model.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Stores the results of the models algorithms.
        Data<Scalar, nq> data;

        /**
         * @brief Initialize the link tree in the model.
         */
        void init_link_tree() {
            // Initialize the joint count to zero
            n_q = 0;

            // Iterate over each joint in the model
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
                if (child_link.idx == -1) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Child link [" << child_link_name << "] of joint ["
                              << joint.name << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                auto parent_link = get_link(parent_link_name);
                if (parent_link.idx == -1) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Parent link [" << parent_link_name << "] of joint ["
                              << joint.name << "] not found";
                    throw std::runtime_error(error_msg.str());
                }

                // Set the parent link index for the child link and add the child link index to the parent link's list
                // of child link indices
                child_link.parent = parent_link.idx;
                parent_link.add_child_link_idx(child_link.idx);

                // If the joint is of type REVOLUTE or PRISMATIC, add its index in the configuration vector and
                // increment the configuration vector size
                if (joint.type == JointType::REVOLUTE || joint.type == JointType::PRISMATIC) {
                    joint.idx = n_q;
                    n_q++;
                }

                // Associate the joint with the child link and update the child link in the links vector
                child_link.joint      = joint;
                links[child_link.idx] = child_link;

                // Update the parent link in the links vector
                links[parent_link.idx] = parent_link;
            }

            // Find the base link of the model by finding the link with no parent link
            for (auto link : links) {
                bool found = false;
                if (link.parent == -1) {
                    if (found) {
                        throw std::runtime_error(
                            "Error! Multiple base links found. The urdf does not contain a valid link tree.");
                    }
                    base_link_idx = link.idx;
                    found         = true;
                }
            }
        }

        /**
         * @brief Updates dynamic links with any fixed joints associated with them, and updates the indices of the
         * dynamic links and their parents.
         */
        void init_dynamics() {
            for (auto& link : links) {
                // If the link has a fixed joint, update the transforms of the child links and its parents link inertia
                if (link.joint.type == JointType::FIXED && link.idx != base_link_idx) {
                    // Update fixed transforms of the child links
                    for (auto child_link_idx : link.child_links) {
                        auto child_link = links[child_link_idx];
                        for (int j = 0; j < links.size(); j++) {
                            if (links[j].name == child_link.name) {
                                // TODO: IMPLEMENT SOMETHING LIKE THIS
                                links[j].joint.X = links[j].joint.X * link.joint.X;
                                break;
                            }
                        }
                    }
                    // Combine spatial inertias
                    auto parent_link = links[link.parent];
                    // Add the spatial inertia of the link to its parent link in the link tree
                    for (int j = 0; j < links.size(); j++) {
                        if (links[j].name == parent_link.name) {
                            Eigen::Matrix<Scalar, 6, 6> X_T = links[j].joint.X;
                            links[j].I += X_T.transpose() * link.I * X_T;
                            break;
                        }
                    }
                }
                else if (link.idx != base_link_idx) {
                    // Add the index of this actuatable link to the list of dynamic link indices
                    q_idx.push_back(link.idx);
                }
            }
            // For each dynamic link, find the index of its parent link in the dynamic link indices
            for (int i = 0; i < q_idx.size(); i++) {
                // Get the dynamic link
                auto link = links[q_idx[i]];
                // Get the parent link which isn't fixed
                auto parent_link = links[link.parent];
                while (parent_link.joint.type == JointType::FIXED) {
                    parent_link = links[parent_link.parent];
                }
                // First check to see if the parent link is the base link
                if (parent_link.idx == base_link_idx) {
                    // Add the index of the base link to the list of parent indices
                    parent.push_back(-1);
                }
                else {
                    // Find the parent link in the q_idx
                    bool found = false;
                    for (int j = 0; j < q_idx.size(); j++) {
                        if (q_idx[j] == parent_link.idx) {
                            parent.push_back(j);
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        parent.push_back(-1);
                    }
                }
            }
        }

        /**
         * @brief Get a link in the model by name.
         * @param name Name of the link.
         * @return Link in the model.
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
         * @brief Get the parent link of a link in the model by name.
         * @param link_idx Index of the link in the model.
         * @return Parent link of the link.
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
         * @brief Get the joint in the model by name.
         * @param name Name of the joint.
         * @return Joint in the model.
         *
         */
        Joint<Scalar> get_joint(const std::string& name) const {
            for (auto link : links) {
                if (link.joint.name == name) {
                    return link.joint;
                }
            }
            // No joint was found, return an empty joint
            return Joint<Scalar>();
        }

        /**
         * @brief Display details of the model.
         */
        void show_details() {
            int spacing = 25;
            std::cout << "| ************************************************ Model Details "
                         "************************************************ |"
                      << std::endl;
            std::cout << "Name : " << name << std::endl;
            std::cout << "No. of links : " << links.size() << std::endl;
            std::cout << "No. of joints : " << joints.size() << std::endl;
            std::cout << "No. of actuatable joints : " << n_q << std::endl;
            std::cout << "Base link name : " << links[base_link_idx].name << std::endl;
            std::cout << "Index" << std::setw(spacing) << "Link Name" << std::setw(spacing) << "Joint Name [idx]"
                      << std::setw(15) << "Joint Type" << std::setw(spacing) << "Parent Name [idx]"
                      << std::setw(spacing) << "Children Names" << std::endl;
            for (auto& link : links) {
                std::string children_names = "";
                for (auto& child_link_idx : link.child_links) {
                    children_names += links[child_link_idx].name + " ";
                }
                if (link.joint.idx != -1) {
                    std::cout << link.idx << std::setw(spacing) << link.name << std::setw(spacing) << link.joint.name
                              << "[" << link.joint.idx << "]" << std::setw(15) << int(link.joint.type)
                              << std::setw(spacing) << links[link.parent].name << "[" << link.parent << "]"
                              << std::setw(spacing) << children_names << std::endl
                              << std::endl;
                }
            }
        }


        /**
         * @brief Get a configuration vector for the model of all zeros.
         * @return Configuration vector of all zeros.
         */
        Eigen::Matrix<Scalar, nq, 1> home_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, nq, 1> q(n_q);
            q.setZero();
            return q;
        }

        /**
         * @brief Get a random configuration vector for the model.
         * @return Random configuration vector.
         */
        Eigen::Matrix<Scalar, nq, 1> random_configuration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, nq, 1> q(n_q);
            q.setRandom();
            q = M_PI * q;
            return q;
        }


        /**
         * @brief Casts the model to a new scalar type.
         * @tparam NewScalar scalar type to cast the model to.
         * @return Model with new scalar type.
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
}  // namespace tr

#endif