#ifndef RML_MODEL_HPP
#define RML_MODEL_HPP

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

        /// @brief Vector of indices of dynamic links in the robot model (links with joints that can be actuated)
        std::vector<int> q_idx = {};

        /// @brief Vector of parent link indices of the associated dynamic link in the dynamic_idxs vector.
        std::vector<int> parent = {};

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

            // Find the base link of the robot model by finding the link with no parent link
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

        /// @brief Updates dynamic links with any fixed joints associated with them, and updates the indices of the
        /// dynamic links and their parents.
        void init_dynamics() {
            for (auto& link : links) {
                // If the link has a fixed joint, update the transforms of the child links and its parents link inertia
                if (link.joint.type == JointType::FIXED && link.idx != base_link_idx) {
                    // Update fixed transforms of the child links
                    for (auto child_link_idx : link.child_links) {
                        auto child_link = links[child_link_idx];
                        for (int j = 0; j < links.size(); j++) {
                            if (links[j].name == child_link.name) {
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

            // Sort q_idx vector based on parent indices
            std::vector<int> indices(q_idx.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](int a, int b) { return parent[a] < parent[b]; });
            std::vector<int> q_idx_sorted(q_idx.size());
            std::vector<int> parent_sorted(q_idx.size());
            for (int i = 0; i < q_idx.size(); i++) {
                q_idx_sorted[i]  = q_idx[indices[i]];
                parent_sorted[i] = parent[indices[i]];
            }
            q_idx  = q_idx_sorted;
            parent = parent_sorted;
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
            for (auto link : links) {
                if (link.joint.name == name) {
                    return link.joint;
                }
            }
            // No joint was found
            return Joint<Scalar>();
        }

        /**
         * @brief Display details of the robot model.
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
         * @brief Display details of the robot models dynamic tree.
         */
        void show_dynamic_details() {
            std::cout << "name = " << name << std::endl;
            std::cout << "q_idx.size() = " << q_idx.size() << std::endl;
            std::cout << "parent.size() = " << parent.size() << std::endl;
            std::cout << "[";
            for (int i = 0; i < q_idx.size(); i++) {
                std::cout << q_idx[i] << ",";
            }
            std::cout << "]" << std::endl;
            std::cout << "[";
            for (int i = 0; i < parent.size(); i++) {
                std::cout << parent[i] << ",";
            }
            std::cout << "]" << std::endl;

            std::cout << "[";
            for (int i = 0; i < q_idx.size(); i++) {
                std::cout << links[q_idx[i]].name << ",";
            }
            std::cout << "]" << std::endl;
            std::cout << "[";
            for (int i = 0; i < parent.size(); i++) {
                if (parent[i] == -1) {
                    std::cout << "base,";
                }
                else {
                    std::cout << links[q_idx[parent[i]]].name << ",";
                }
            }
            std::cout << "]" << std::endl;

            for (int i = 0; i < q_idx.size(); i++) {
                std::cout << links[q_idx[i]].joint.X << std::endl;
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