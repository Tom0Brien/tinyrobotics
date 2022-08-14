#ifndef RML_FORWARDKINEMATICS_HPP
#define RML_FORWARDKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

#include "Model.hpp"

namespace RML {

    /**
     * @brief Computes the transform between two links.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics(const Model<Scalar>& model,
                                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                  const std::string& source_link_name,
                                                                  const std::string& target_link_name) {
        std::shared_ptr<Link<Scalar>> current_link = model.get_link(source_link_name);

        // Build kinematic tree from source {s} to base {b} frame
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hbs = forward_kinematics(model, q, source_link_name);
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hsb = RML::inv(Hbs);

        // Build kinematic tree from base {b} to target {t} frame
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hbt = forward_kinematics(model, q, target_link_name);

        // Compute transform between source {s} and target {t} frames
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hst = Hsb * Hbt;
        return Hst;
    }

    /**
     * @brief Computes the transform between base link to target.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param target_link_name {s} The link from which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics(const Model<Scalar>& model,
                                                                  const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                  const std::string& target_link_name) {
        std::shared_ptr<Link<Scalar>> current_link = model.get_link(target_link_name);

        // Check if the link is within the kinematic tree
        if (current_link == nullptr) {
            std::string error_msg = "Error: Link " + target_link_name + " not found.";
            throw std::runtime_error(error_msg);
        }

        // Build kinematic tree from target_link {t} to base {b}
        Eigen::Transform<Scalar, 3, Eigen::Affine> Htb = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
        while (current_link->name != model.base_link->name) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> H = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
            if (current_link->joint->type == JointType::REVOLUTE) {
                Scalar q_current = q(current_link->joint->q_index);
                // Rotate by q_current around axis
                H.linear() = Eigen::AngleAxis<Scalar>(q_current, current_link->joint->axis).toRotationMatrix();
            }
            else if (current_link->joint->type == JointType::PRISMATIC) {
                Scalar q_current = q(current_link->joint->q_index);
                // Translate by q_current along axis
                H.translation() = current_link->joint->axis * q_current;
            }
            Htb = Htb * RML::inv(H);
            // Apply inverse joint transform as we are going back up tree
            Htb = Htb * RML::inv(current_link->joint->parent_transform);
            // Move up the tree to parent
            current_link = current_link->parent_link;
        }

        // Return transform from base {b} to target {t}
        return RML::inv(Htb);
    }

    /**
     * @brief Computes the transform between source link {s} and target {t} centre of mass (CoM) {c}.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The transform between the two links.
     */
    template <typename Scalar, int nq>
    Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics_com(Model<Scalar>& model,
                                                                      const Eigen::Matrix<Scalar, nq, 1>& q,
                                                                      std::string& source_link_name,
                                                                      std::string& target_link_name) {
        // Assert the configuration vector is valid
        // assert(q.size() == model.n_q); TODO: FIX THIS FOR AUTODIFF

        // Compute forward kinematics from source {b} to target {t}
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);

        // Compute forward kinematics from source {s} to CoM {c}
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hsc = Hst * model.get_link(target_link_name)->centre_of_mass;

        // Return transform from source {s} to CoM {c}
        return Hsc;
    }

    /**
     * @brief Computes the position of the target link in the source link frame.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The configuration vector of the robot model which achieves the desired pose.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> position(Model<Scalar>& model,
                                         const Eigen::Matrix<Scalar, nq, 1>& q,
                                         std::string& source_link_name,
                                         std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        Eigen::Matrix<Scalar, 3, 1> rTSs(Hst.translation());
        return rTSs;
    }

    /**
     * @brief Computes the orientation of the target link in the source link frame expressed in euler angles. [roll,
     * pitch, yaw]
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The rotation matrix between the source and target link
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 3> orientation(Model<Scalar>& model,
                                            const Eigen::Matrix<Scalar, nq, 1>& q,
                                            std::string& source_link_name,
                                            std::string& target_link_name) {
        Eigen::Transform<Scalar, 3, Eigen::Affine> Hst =
            forward_kinematics(model, q, source_link_name, target_link_name);
        return Hst.linear();
    }

    /**
     * @brief Computes translation component of the geometric Jacobian between two links Jv.
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The translation component of the geometric Jacobian between two links.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, nq> Jv(Model<Scalar>& model,
                                    Eigen::Matrix<Scalar, nq, 1>& q,
                                    std::string& source_link_name,
                                    std::string& target_link_name) {

        // Assert the configuration vector is valid
        assert(q.size() == model.n_q);

        // Cast q and model to autodiff type
        Eigen::Matrix<autodiff::real, nq, 1> q_real(q);  // the input vector q
        RML::Model<autodiff::real> autodiff_model;
        autodiff_model = model.template cast<autodiff::real>();
        // The output vector F = f(x) evaluated together with Jacobian matrix below
        Eigen::Matrix<autodiff::real, 3, 1> F;
        // Evaluate the output vector F and the Jacobian matrix dF/dx
        Eigen::MatrixXd Jv = jacobian(position<autodiff::real, nq>,
                                      wrt(q_real),
                                      at(autodiff_model, q_real, source_link_name, target_link_name),
                                      F);
        return Jv;
    }

    /**
     * @brief Computes the geometric Jacobian between the base and the target link
     * @param model The robot model.
     * @param q The joint configuration of the robot.
     * @param source_link_name {s} The link from which the transform is computed.
     * @param target_link_name {t} The link to which the transform is computed.
     * @return The geometric jacobian between the base and the target link.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 6, nq> geometric_jacobian(Model<Scalar>& model,
                                                    Eigen::Matrix<Scalar, nq, 1>& q,
                                                    std::string& target_link_name) {
        // Setup the geometric jabobian matrix
        Eigen::Matrix<Scalar, 6, nq> J = Eigen::Matrix<Scalar, 6, nq>::Zero();

        // Start at the target link
        std::shared_ptr<Link<Scalar>> current_link = model.get_link(target_link_name);

        // Compute the displacement of the target link in the base link frame
        Eigen::Matrix<Scalar, 3, 1> rTBb = position(model, q, model.base_link->name, target_link_name);

        while (current_link->name != model.base_link->name) {
            if (current_link->joint != nullptr) {
                // Compute the rotation matrix between base and link
                Eigen::Transform<Scalar, 3, Eigen::Affine> Hbim1 =
                    forward_kinematics(model, q, model.base_link->name, current_link->parent_link->name);
                if (current_link->joint->type == JointType::PRISMATIC) {
                    J.block(0, current_link->joint->q_index, 3, 1) = Hbim1.linear() * current_link->joint->axis;
                    J.block(3, current_link->joint->q_index, 3, 1) = Eigen::Matrix<Scalar, 3, 1>::Zero();
                }
                else if (current_link->joint->type == JointType::REVOLUTE) {
                    // Rotate the axis into the base frame
                    J.block(0, current_link->joint->q_index, 3, 1) =
                        (Hbim1.linear() * current_link->joint->axis).cross(rTBb - Hbim1.translation());
                    J.block(3, current_link->joint->q_index, 3, 1) = Hbim1.linear() * current_link->joint->axis;
                }
            }
            // Move up the tree to parent
            current_link = current_link->parent_link;
        }

        return J;
    }

    /**
     * @brief Computes the centre of mass expressed in source link frame.
     * @param model The robot model.
     * @param q The configuration vector of the robot model.
     * @param source_link_name The link from which the centre of mass position is computed.
     *
     * @return The centre of mass position expressed in source link frame.
     */
    template <typename Scalar, int nq>
    Eigen::Matrix<Scalar, 3, 1> centre_of_mass(Model<Scalar>& model,
                                               const Eigen::Matrix<Scalar, nq, 1>& q,
                                               std::string& source_link_name) {

        // Assert the configuration vector is valid
        assert(q.size() == model.n_q);

        // For each link in the model, compute the transform from the source link to the CoM of the link
        Scalar total_mass                = 0;
        Eigen::Matrix<Scalar, 3, 1> rISs = Eigen::Matrix<Scalar, 3, 1>::Zero();
        for (auto link : model.links) {
            // Compute the transform from the source link to the CoM of the link
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hsc =
                forward_kinematics_com(model, q, source_link_name, link->name);
            // Compute the centre of mass of the link
            rISs = rISs + Hsc.translation() * link->mass;
            // Add the links mass to the total mass
            total_mass += link->mass;
        }
        // Compute the centre of mass  from the source link
        return rISs / total_mass;
    }

}  // namespace RML

#endif
