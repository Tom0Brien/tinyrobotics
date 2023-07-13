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

#include "joint.hpp"
#include "link.hpp"

/** \file model.hpp
 * @brief Contains struct for representing a tinyrobotics model.
 */
namespace tinyrobotics {

    /**
     * @brief A tinyrobotics model.
     * @details A tinyrobotics model is a collection of links and joints that represents a robot.
     * @tparam Scalar The scalar type for the model.
     * @tparam nq The number of configuration coordinates (number of degrees of freedom).
     */
    template <typename Scalar, int nq>
    struct Model {
        /// @brief Name of the model
        std::string name = "";

        /// @brief Number of configuration coordinates (degrees of freedom) in the model
        int n_q = 0;

        /// @brief Index of the base link in the models links vector
        int base_link_idx = -1;

        /// @brief Map to indices of links in the models link vector that have a non-fixed joints
        std::vector<int> q_map = {};

        /// @brief Vector of parent link indices of the links which have a non-fixed joints
        std::vector<int> parent = {};

        /// @brief Gravitational acceleration vector experienced by model
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Total mass of the model
        Scalar mass = 0;

        /// @brief Vector of links in the model
        std::vector<Link<Scalar>> links = {};

        /// @brief Mass matrix
        Eigen::Matrix<Scalar, nq, nq> mass_matrix = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Kinetic energy
        Scalar kinetic_energy = 0;

        /// @brief Potential energy
        Scalar potential_energy = 0;

        /// @brief Total_energy
        Scalar total_energy = 0;

        /// @brief Vector of forward kinematics data
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics = {};

        /// @brief Vector of forward kinematics com data
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics_com = {};

        /// @brief Joint acceleration.
        Eigen::Matrix<Scalar, nq, 1> ddq = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// @brief Joint torque/force.
        Eigen::Matrix<Scalar, nq, 1> tau = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// **************** Pre-allcoated variables for dynamics algorithms ****************

        /// @brief Spatial transforms from parent to child links
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xup =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Motion subspace matrices for the joints
        std::vector<Eigen::Matrix<Scalar, 6, 1>> S =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial velocities of the robot links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> v =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial acceleration bias terms for the robot links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> c =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Articulated-body inertia matrices for the robot links
        std::vector<Eigen::Matrix<Scalar, 6, 6>> IA =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Articulated-body forces for the robot links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> pA =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial force projections for the joints
        std::vector<Eigen::Matrix<Scalar, 6, 1>> U =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Joint force inertia terms for the robot links
        std::vector<Scalar> d = std::vector<Scalar>(nq, 0);

        /// @brief Joint force bias terms for the robot links
        std::vector<Scalar> u = std::vector<Scalar>(nq, 0);

        /// @brief Spatial accelerations of the robot links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> a =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Homogeneous transformation matrix
        Eigen::Transform<Scalar, 3, Eigen::Isometry> T = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 1>> avp =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 1>> fvp =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        Eigen::Matrix<Scalar, nq, 1> C = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// @brief
        Eigen::Matrix<Scalar, 6, 1> fh = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /// @brief
        std::vector<Eigen::Matrix<Scalar, 6, 6>> IC =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Gravity vector in spatial coordinates
        Eigen::Matrix<Scalar, 6, 1> spatial_gravity = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /// @brief Geometric Jacobian
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        /// @brief Hessian Product H*dq
        Eigen::Matrix<Scalar, 6, nq> H_dq = Eigen::Matrix<Scalar, 6, nq>::Zero();

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
            std::cout << "No. of actuated joints : " << n_q << std::endl;
            std::cout << "Base link name         : " << links[base_link_idx].name << std::endl;
            std::cout << "Model Mass             : " << mass << std::endl;
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
            new_model.n_q                  = n_q;
            new_model.base_link_idx        = base_link_idx;
            new_model.gravity              = gravity.template cast<NewScalar>();
            new_model.mass                 = NewScalar(mass);
            for (auto& link : links) {
                new_model.links.push_back(link.template cast<NewScalar>());
            }
            new_model.mass_matrix      = mass_matrix.template cast<NewScalar>();
            new_model.kinetic_energy   = NewScalar(kinetic_energy);
            new_model.potential_energy = NewScalar(potential_energy);
            new_model.total_energy     = NewScalar(total_energy);
            new_model.C                = C.template cast<NewScalar>();
            new_model.fh               = fh.template cast<NewScalar>();
            new_model.spatial_gravity  = spatial_gravity.template cast<NewScalar>();
            new_model.J                = J.template cast<NewScalar>();
            new_model.forward_kinematics.resize(forward_kinematics.size());
            new_model.forward_kinematics_com.resize(forward_kinematics_com.size());
            for (int i = 0; i < forward_kinematics.size(); i++) {
                new_model.forward_kinematics[i] = forward_kinematics[i].template cast<NewScalar>();
            }
            for (int i = 0; i < forward_kinematics_com.size(); i++) {
                new_model.forward_kinematics_com[i] = forward_kinematics_com[i].template cast<NewScalar>();
            }
            for (int i = 0; i < Xup.size(); i++) {
                new_model.Xup[i] = Xup[i].template cast<NewScalar>();
                new_model.S[i]   = S[i].template cast<NewScalar>();
                new_model.v[i]   = v[i].template cast<NewScalar>();
                new_model.c[i]   = c[i].template cast<NewScalar>();
                new_model.IA[i]  = IA[i].template cast<NewScalar>();
                new_model.pA[i]  = pA[i].template cast<NewScalar>();
                new_model.U[i]   = U[i].template cast<NewScalar>();
                new_model.d[i]   = NewScalar(d[i]);
                new_model.u[i]   = NewScalar(u[i]);
                new_model.a[i]   = a[i].template cast<NewScalar>();
                new_model.avp[i] = avp[i].template cast<NewScalar>();
                new_model.fvp[i] = fvp[i].template cast<NewScalar>();
                new_model.IC[i]  = IC[i].template cast<NewScalar>();
            }
            return new_model;
        }
    };
}  // namespace tinyrobotics

#endif