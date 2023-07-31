#ifndef TR_MODEL_HPP
#define TR_MODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "inversekinematics.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace tinyrobotics {

    /**
     * @brief A tinyrobotics model.
     * @details A tinyrobotics model is a collection of links and joints that represents a robot.
     * @tparam Scalar The scalar type for the model.
     */
    template <typename Scalar, int nq>
    struct Model {

        // ******************************** Model ********************************

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

        /// @brief Vector of links in the model
        std::vector<Link<Scalar>> links = {};

        /**
         * @brief Get a link in the model by name.
         * @param name Name of the link.
         * @return Link in the model.
         */
        Link<Scalar> getLink(const std::string& name) const {
            for (auto link : links) {
                if (link.name == name) {
                    return link;
                }
            }
            // No link was found
            return Link<Scalar>();
        }

        /**
         * @brief Get the joint in the model by name.
         * @param name Name of the joint.
         * @return Joint in the model.
         *
         */
        Joint<Scalar> getJoint(const std::string& name) const {
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
        void showDetails() {
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
        Eigen::Matrix<Scalar, nq, 1> homeConfiguration() const {
            assert(n_q > 0);
            Eigen::Matrix<Scalar, nq, 1> q(n_q);
            q.setZero();
            return q;
        }

        /**
         * @brief Get a random configuration vector for the model.
         * @return Random configuration vector.
         */
        Eigen::Matrix<Scalar, nq, 1> randomConfiguration() const {
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

        // ******************************** Kinematics ********************************

        /// @brief Vector of forward kinematics transforms for all links
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics = {};

        /// @brief Vector of forward kinematics transforms for all links centre of masses
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forward_kinematics_com = {};

        /// @brief Jacobian
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        /// @brief Centre of mass of the model
        Eigen::Matrix<Scalar, 3, 1> centre_of_mass = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /**
         * @brief Retrieves the index of the target link in the tinyrobotics model.
         * @param model tinyrobotics model.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @return Index of the target link in the model.
         * @throws std::invalid_argument if the TargetLink type is not int or std::string.
         */
        template <typename TargetLink>
        int getLinkIndex(const TargetLink& target_link);

        /**
         * @brief Computes the transform to all the links in the tinyrobotics model.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @return Stores the transform to all the links in forward_kinematics.
         */
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forwardKinematics(
            const Eigen::Matrix<Scalar, nq, 1>& q);

        /**
         * @brief Computes the transform between target and the base link. The transform converts points in target
         * frame to the base link frame.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @return Homogeneous transform between the target and the base link.
         */
        template <typename TargetLink>
        Eigen::Transform<Scalar, 3, Eigen::Isometry> forwardKinematics(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                       const TargetLink& target_link);

        /**
         * @brief Computes the transform between target and the source link. The transform converts points in target
         * frame to the source link frame.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @param source_link Source link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
         * @return Homogeneous transform between the target and the source link.
         */
        template <typename TargetLink, typename SourceLink>
        Eigen::Transform<Scalar, 3, Eigen::Isometry> forwardKinematics(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                       const TargetLink& target_link,
                                                                       const SourceLink& source_link);


        /**
         * @brief Computes the transform between centre of mass of each link and the source link.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @return Vector of homogeneous transforms between the centre of mass of each link and the source link.
         */
        std::vector<Eigen::Transform<Scalar, 3, Eigen::Isometry>> forwardKinematicsCOM(
            const Eigen::Matrix<Scalar, nq, 1>& q);

        /**
         * @brief Computes the transform between source link and target centre of mass. The transform converts points in
         * the target links centre of mass frame into the source link frame.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @param source_link Source link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
         * @return Homogeneous transform from source link to target link centre of mass.
         */
        template <typename TargetLink, typename SourceLink>
        Eigen::Transform<Scalar, 3, Eigen::Isometry> forwardKinematicsCOM(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                          const TargetLink& target_link,
                                                                          const SourceLink& source_link = 0);

        /**
         * @brief Computes the geometric jacobian of the target link from the base link, in the base link frame.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @return The geometric jacobian of the target link from the base link, in the base link frame.
         */
        template <typename TargetLink>
        Eigen::Matrix<Scalar, 6, nq> jacobian(const Eigen::Matrix<Scalar, nq, 1>& q, const TargetLink& target_link);

        /**
         * @brief Computes the geometric jacobian of the target links from the base link, in the base link frame.
         * @param model tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param target_link Target link, which can be an integer (index) or a string (name).
         * @tparam TargetLink Type of target_link parameter, which can be int or std::string.
         * @return The geometric jacobian of the target link from the base link in the base link frame.
         */
        template <typename TargetLink>
        Eigen::Matrix<Scalar, 6, nq> jacobianCOM(const Eigen::Matrix<Scalar, nq, 1>& q, const TargetLink& target_link);


        /**
         * @brief Computes the centre of mass expressed in source link frame.
         * @param model tinyrobotics model.
         * @param q The configuration vector of the robot model.
         * @param source_link Source link, which can be an integer (index) or a string (name), from which the centre of
         * mass is expressed.
         * @tparam SourceLink Type of source_link parameter, which can be int or std::string.
         * @return The centre of mass position expressed in source link frame.
         */
        template <typename SourceLink = int>
        Eigen::Matrix<Scalar, 3, 1> centreOfMass(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                 const SourceLink& source_link = 0);

        /**
         * @brief Solves the inverse kinematics problem between two links using user specified method.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematics(const std::string& target_link_name,
                                                       const std::string& source_link_name,
                                                       const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                                                       const Eigen::Matrix<Scalar, nq, 1> q0,
                                                       const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Solves the inverse kinematics problem between two links using Broyden-Fletcher-Goldfarb-Shanno (BFGS).
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematicsBFGS(
            const std::string& target_link_name,
            const std::string& source_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
            const Eigen::Matrix<Scalar, nq, 1> q0,
            const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Solves the inverse kinematics problem between two links using Particle Swarm Optimization.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematicsPSO(
            const std::string& target_link_name,
            const std::string& source_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
            const Eigen::Matrix<Scalar, nq, 1> q0,
            const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Solves the inverse kinematics problem between two links using the Levenberg-Marquardt method.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematicsLevenbergMarquardt(
            const std::string& target_link_name,
            const std::string& source_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
            const Eigen::Matrix<Scalar, nq, 1> q0,
            const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Solves the inverse kinematics problem between two links using the Jacobian method.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematicsJacobian(
            const std::string& target_link_name,
            const std::string& source_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
            const Eigen::Matrix<Scalar, nq, 1> q0,
            const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Solves the inverse kinematics problem between two links using NLopt.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 The initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, nq, 1> inverseKinematicsNLOPT(
            const std::string& target_link_name,
            const std::string& source_link_name,
            const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
            const Eigen::Matrix<Scalar, nq, 1> q0,
            const InverseKinematicsOptions<Scalar, nq>& options);

        /**
         * @brief Cost function for general inverse kinematics with analytical jacobian.
         * @param model tinyrobotics model.
         * @param target_link_name {t} Link to which the transform is computed.
         * @param source_link_name {s} Link from which the transform is computed.
         * @param desired_pose Desired pose of the target link in the source link frame.
         * @param q0 Initial guess for the configuration vector.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Scalar cost(const Eigen::Matrix<Scalar, nq, 1>& q,
                    const std::string& target_link_name,
                    const std::string& source_link_name,
                    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose,
                    const Eigen::Matrix<Scalar, nq, 1>& q0,
                    Eigen::Matrix<Scalar, nq, 1>& gradient,
                    const InverseKinematicsOptions<Scalar, nq>& options);


        // ********************************  Dynamics ********************************

        /// @brief Gravitational acceleration vector experienced by model
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

        /// @brief Gravitational acceleration vector in spatial coordinates
        Eigen::Matrix<Scalar, 6, 1> spatial_gravity = Eigen::Matrix<Scalar, 6, 1>::Zero();

        /// @brief Total mass of the model
        Scalar mass = 0;

        /// @brief Joint accelerations
        Eigen::Matrix<Scalar, nq, 1> ddq = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// @brief Joint torques/forces
        Eigen::Matrix<Scalar, nq, 1> tau = Eigen::Matrix<Scalar, nq, 1>::Zero();

        /// @brief Mass matrix
        Eigen::Matrix<Scalar, nq, nq> mass_matrix = Eigen::Matrix<Scalar, nq, nq>::Zero();

        /// @brief Kinetic energy
        Scalar kinetic_energy = 0;

        /// @brief Potential energy
        Scalar potential_energy = 0;

        /// @brief Total_energy
        Scalar total_energy = 0;

        /// @brief Spatial transforms from parent to child links
        std::vector<Eigen::Matrix<Scalar, 6, 6>> Xup =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Motion subspace matrices for the joints
        std::vector<Eigen::Matrix<Scalar, 6, 1>> S =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial velocities of the models links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> v =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial acceleration bias terms for the models links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> c =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Articulated-body inertia matrices for the models links
        std::vector<Eigen::Matrix<Scalar, 6, 6>> IA =
            std::vector<Eigen::Matrix<Scalar, 6, 6>>(nq, Eigen::Matrix<Scalar, 6, 6>::Zero());

        /// @brief Articulated-body forces for the models links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> pA =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Spatial force projections for the joints
        std::vector<Eigen::Matrix<Scalar, 6, 1>> U =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief Joint force inertia terms for the models links
        std::vector<Scalar> d = std::vector<Scalar>(nq, 0);

        /// @brief Joint force bias terms for the models links
        std::vector<Scalar> u = std::vector<Scalar>(nq, 0);

        /// @brief Spatial accelerations of the models links
        std::vector<Eigen::Matrix<Scalar, 6, 1>> a =
            std::vector<Eigen::Matrix<Scalar, 6, 1>>(nq, Eigen::Matrix<Scalar, 6, 1>::Zero());

        /// @brief
        Eigen::Matrix<Scalar, 6, 1> vJ = Eigen::Matrix<Scalar, 6, 1>::Zero();

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

        /**
         * @brief Compute the mass matrix of the tinyrobotics model.
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         */
        Eigen::Matrix<Scalar, nq, nq> massMatrix(const Eigen::Matrix<Scalar, nq, 1>& q);

        /**
         * @brief Compute the kinetic_energy of the tinyrobotics model.
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param dq The joint velocity of the robot.
         */
        Scalar kineticEnergy(const Eigen::Matrix<Scalar, nq, 1>& q, const Eigen::Matrix<Scalar, nq, 1>& dq);

        /**
         * @brief Compute the potentialEnergy of the tinyrobotics model.
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         */
        Scalar potentialEnergy(const Eigen::Matrix<Scalar, nq, 1>& q);

        /**
         * @brief Compute the total energy of the tinyrobotics model.
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param dq Joint velocity of the robot.
         */
        Scalar totalEnergy(const Eigen::Matrix<Scalar, nq, 1>& q, const Eigen::Matrix<Scalar, nq, 1>& dq);

        /**
         * @brief Apply external forces to the tinyrobotics model
         * @param m tinyrobotics model.
         * @param Xup The spatial transformation matrices between the ith link and its parent.
         * @param f_in The input force array of the tinyrobotics model.
         * @param f_external The external force array to be added to the input force array.
         * @return f_out The output force array with the external forces incorporated.
         */
        std::vector<Eigen::Matrix<Scalar, 6, 1>> applyExternalForces(
            const std::vector<Eigen::Matrix<Scalar, 6, 6>>& Xup,
            const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_in,
            const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {});

        /**
         * @brief Compute the forward dynamics of the tinyrobotics model via Articulated-Body Algorithm
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param qd Joint velocity of the robot.
         * @param tau Joint torque of the robot.
         * @param f_external External forces acting on the robot.
         * @return Joint accelerations of the model.
         */
        Eigen::Matrix<Scalar, nq, 1> forwardDynamics(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                     const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                     const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                     const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {});

        /**
         * @brief Compute the forward dynamics of the tinyrobotics model via Composite-Rigid-Body Algorithm
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param qd Joint velocity of the robot.
         * @param tau Joint torque of the robot.
         * @param f_external External forces acting on the robot.
         * @return Joint accelerations of the model.
         */
        Eigen::Matrix<Scalar, nq, 1> forwardDynamicsCRB(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                        const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                        const Eigen::Matrix<Scalar, nq, 1>& tau,
                                                        const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {});

        /**
         * @brief Compute the inverse dynamics of a tinyrobotics model
         * @param m tinyrobotics model.
         * @param q Joint configuration of the robot.
         * @param qd Joint velocity of the robot.
         * @param f_external External forces acting on the robot.
         * @return tau
         */
        Eigen::Matrix<Scalar, nq, 1> inverseDynamics(const Eigen::Matrix<Scalar, nq, 1>& q,
                                                     const Eigen::Matrix<Scalar, nq, 1>& qd,
                                                     const Eigen::Matrix<Scalar, nq, 1>& qdd,
                                                     const std::vector<Eigen::Matrix<Scalar, 6, 1>>& f_ext = {});
    };
}  // namespace tinyrobotics
#include "dynamics.hxx"
#include "inversekinematics.hxx"
#include "kinematics.hxx"
#endif