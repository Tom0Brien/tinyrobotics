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

/** \file Model.hpp
 * @brief Contains struct for representing a tinyrobotics model.
 */
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

        /// @brief Vector of indices of links in the models link vector that have a non-fixed joint.
        std::vector<int> q_idx = {};

        /// @brief Vector of parent link indices of the links which are non-fixed.
        std::vector<int> parent = {};

        /// @brief Vector of joints in the model.
        std::vector<Joint<Scalar>> joints = {};

        /// @brief Gravitational acceleration experienced by model.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Stores the results of the models algorithms.
        Data<Scalar, nq> data;

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
            const int spacing = 25;
            const std::string separator(150, '*');
            std::cout << separator << std::endl;
            std::cout << "Model name             : " << name << std::endl;
            std::cout << "No. of links           : " << links.size() << std::endl;
            std::cout << "No. of joints          : " << joints.size() << std::endl;
            std::cout << "No. of actuated joints : " << n_q << std::endl;
            std::cout << "Base link name         : " << links[base_link_idx].name << std::endl << std::endl;
            std::cout << std::left << std::setw(spacing) << "Index" << std::setw(spacing) << "Link Name"
                      << std::setw(spacing) << "Joint Name [idx]" << std::setw(spacing) << "Joint Type"
                      << std::setw(spacing) << "Parent Name [idx]" << std::setw(spacing) << "Children Names"
                      << std::endl;
            for (const auto& link : links) {
                std::string children_names;
                for (const auto& child_link_idx : link.child_links) {
                    children_names += links[child_link_idx].name + " ";
                }
                Link<Scalar> parent_link;
                std::string parent_name = "";
                int parent_idx          = -1;
                if (link.parent != -1) {
                    parent_link = links[link.parent];
                    parent_name = parent_link.name;
                    parent_idx  = link.parent;
                }
                std::cout << std::left << std::setw(spacing) << link.idx << std::setw(spacing) << link.name
                          << std::setw(spacing) << link.joint.name + "[" + std::to_string(link.joint.idx) + "]"
                          << std::setw(spacing) << link.joint.get_type() << std::setw(spacing)
                          << parent_name + " [" + std::to_string(parent_idx) + "]" << std::setw(spacing)
                          << children_names << std::endl;
            }
            std::cout << separator << std::endl;
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