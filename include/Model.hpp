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
#include <tinyxml2.h>

#include "Joint.hpp"
#include "Link.hpp"

namespace RML {

    /**
     * @brief A struct for storing the results of model algorithms.
     */
    template <typename Scalar>
    struct Results {

        /// @brief Joint configuration.
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q;

        /// @brief Joint velocity.
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> dq;

        /// @brief Joint acceleration.
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ddq;

        /// @brief Joint torque.
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> tau;

        /// @brief Mass matrix.
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> M;

        /// @brief Coriolis matrix.
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C;

        /// @brief Gravity torque vector.
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> g;

        /// @brief The kinetic co-energy.
        Scalar T = 0;

        /// @brief The potential energy.
        Scalar V = 0;

        /// @brief The hamiltonian (total energy).
        Scalar H = 0;

        /// @brief Vector of forward kinematics results.
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Affine>> FK;

        /**
         * @brief Resize all matrices to the given size.
         * @param n The size to resize all matrices to.
         */
        void resize(int n) {
            q.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            dq.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            ddq.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            tau.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
            M.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            C.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n));
            g.conservativeResizeLike(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n, 1));
        }

        /**
         * @brief Cast to NewScalar type.
         */
        template <typename NewScalar>
        Results<NewScalar> cast() {
            Results<NewScalar> new_res;
            new_res.q   = q.template cast<NewScalar>();
            new_res.dq  = dq.template cast<NewScalar>();
            new_res.ddq = ddq.template cast<NewScalar>();
            new_res.tau = tau.template cast<NewScalar>();
            new_res.M   = M.template cast<NewScalar>();
            new_res.C   = C.template cast<NewScalar>();
            new_res.g   = g.template cast<NewScalar>();
            new_res.T   = NewScalar(T);
            new_res.V   = NewScalar(V);
            new_res.H   = NewScalar(H);
            for (int i = 0; i < FK.size(); i++) {
                new_res.FK[i] = FK[i].template cast<NewScalar>();
            }
            return new_res;
        }
    };

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

        /// @brief The index of the base link in the robot model.
        int base_link_idx = -1;

        /// @brief Vector of links in the robot model.
        std::vector<Link<Scalar>> links = {};

        /// @brief Vector of joints in the robot model.
        std::vector<Joint<Scalar>> joints = {};

        /// @brief The gravitational acceleration experienced by robot.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Stores the results of the models algorithms.
        Results<Scalar> results;

        /**
         * @brief Initialize the link tree of the robot model.
         *
         */
        void init_link_tree(std::map<std::string, std::string>& parent_link_tree) {
            // Set n_q to zero
            n_q = 0;

            for (auto joint : joints) {

                std::string parent_link_name = joint.parent_link_name;
                std::string child_link_name  = joint.child_link_name;

                if (parent_link_name.empty()) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint.name
                              << "] is missing a parent link specification.";
                    throw std::runtime_error(error_msg.str());
                }
                if (child_link_name.empty()) {
                    std::ostringstream error_msg;
                    error_msg << "Error while constructing model! Joint [" << joint.name
                              << "] is missing a child link specification.";
                    throw std::runtime_error(error_msg.str());
                }

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


                child_link.parent_link_idx = parent_link.link_idx;
                child_link.joint_idx       = joint.joint_idx;
                // Overwrite the child link with the updated link
                links[child_link.link_idx] = child_link;

                parent_link.add_child_joint_idx(joint.joint_idx);
                parent_link.add_child_link_idx(child_link.link_idx);

                // Overwrite the parent link with the updated link
                links[parent_link.link_idx] = parent_link;

                parent_link_tree[child_link.name] = parent_link_name;

                // If the joint is actuatable, add its configuration vector q_idx
                if (joint.type == JointType::REVOLUTE || joint.type == JointType::PRISMATIC) {
                    joint.q_idx = n_q;
                    n_q++;
                    // Overwrite the joint with the updated joint
                    joints[joint.joint_idx] = joint;
                }
            }
        }

        /**
         * @brief Find the base link of the robot model.
         *
         */
        void find_base(const std::map<std::string, std::string>& parent_link_tree) {
            for (auto link : links) {
                auto parent = parent_link_tree.find(link.name);
                if (parent == parent_link_tree.end()) {
                    if (base_link_idx == -1) {
                        base_link_idx = get_link_idx(link.name);
                    }
                    else {
                        std::ostringstream error_msg;
                        error_msg << "Error! Multiple base links found: (" << links[base_link_idx].name << ", "
                                  << ") and (" + link.name + ")!";
                        throw std::runtime_error(error_msg.str());
                    }
                }
            }
            if (base_link_idx == -1) {
                throw std::runtime_error("Error! No base link found. The urdf does not contain a valid link tree.");
            }
        }

        /**
         * @brief Get a link in the robot model.
         *
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
                    return links[link.parent_link_idx];
                }
            }
            // No link was found
            throw std::runtime_error("Error! Parent of Link [" + name + "] not found!");
            return Link<Scalar>();
        }

        /**
         * @brief Get the index of a link in the robot model.
         * @param name The name of the link.
         * @return The index of the link in the robot model.
         */
        int get_link_idx(const std::string& name) const {
            for (auto link : links) {
                if (link.name == name) {
                    return link.link_idx;
                }
            }
            // No link was found
            throw std::runtime_error("Error! Link [" + name + "] not found!");
            return -1;
        }

        /**
         * @brief Get the joint in the robot model.
         * @param name The name of the joint.
         * @return The joint in the robot model.
         *
         */
        Joint<Scalar> get_joint(const std::string& name) {
            for (auto joint : joints) {
                if (joint.name == name) {
                    return joints[joint.joint_idx];
                }
            }
            // No joint was found
            return Joint<Scalar>();
        }

        /**
         * @brief Get the index of a joint in the robot model.
         * @param name The name of the joint.
         * @return The index of the joint in the robot model.
         *
         */
        int get_joint_idx(const std::string& name) {
            for (auto joint : joints) {
                if (joint.name == name) {
                    return joint.joint_idx;
                }
            }
            // No joint was found
            return -1;
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
                for (auto& child_joint_idx : link.child_joints) {
                    children_names += joints[child_joint_idx].child_link_name + " ";
                }
                // std::cout << "link.joint_idx : " << link.joint_idx << std::endl;
                if (link.joint_idx != -1) {
                    std::cout << link.link_idx << std::setw(spacing) << link.name << std::setw(spacing)
                              << joints[link.joint_idx].name << std::setw(15) << joints[link.joint_idx].q_idx
                              << std::setw(15) << int(joints[link.joint_idx].type) << std::setw(spacing)
                              << links[link.parent_link_idx].name << std::setw(spacing + 15) << children_names
                              << std::endl
                              << std::endl;
                }
            }
        }

        /**
         * @brief Get a configuration vector for the robot model of all zeros.
         *
         */
        template <int nq>
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
        template <int nq>
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
        Model<NewScalar> cast() {
            Model<NewScalar> new_model = Model<NewScalar>();
            new_model.name             = name;
            new_model.n_links          = n_links;
            new_model.n_joints         = n_joints;
            new_model.n_q              = n_q;
            new_model.base_link_idx    = base_link_idx;
            new_model.gravity          = gravity.template cast<NewScalar>();
            for (auto& joint : joints) {
                new_model.joints.push_back(joint.template cast<NewScalar>());
            }
            for (auto& link : links) {
                new_model.links.push_back(link.template cast<NewScalar>());
            }
            new_model.results = results.template cast<NewScalar>();
            return new_model;
        }
    };
}  // namespace RML

#endif