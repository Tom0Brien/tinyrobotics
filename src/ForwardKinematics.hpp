#ifndef RML_FORWARDKINEMATICS_HPP
#define RML_FORWARDKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <map>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include "RobotModel.hpp"
#include "txml.h"

using namespace autodiff;

namespace RML {

        /**
         * @brief Computes the transform between two links.
         * @param model The robot model.
         * @param q The joint configuration of the robot.
         * @param source_link_name {s} The link from which the transform is computed.
         * @param target_link_name {t} The link to which the transform is computed.
         * @return The transform between the two links.
         */
        template <typename Scalar>
        Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics(const std::shared_ptr<RobotModel<Scalar>> model,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q, const std::string& source_link_name, const std::string& target_link_name) {
            std::shared_ptr<Link<Scalar>> current_link = model->get_link(source_link_name);

            // Build kinematic tree from source {s} to base {b} frame
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hbs = forward_kinematics(model, q, source_link_name);
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hsb = Hbs.inverse();

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
        template <typename Scalar>
        Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics(const std::shared_ptr<RobotModel<Scalar>> model,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q, const std::string& target_link_name) {

            // Assert the configuration vector is valid
            // assert(q.size() == model->n_q); TODO: FIX THIS FOR AUTODIFF

            std::shared_ptr<Link<Scalar>> current_link = model->get_link(target_link_name);

            // Check if the link is within the kinematic tree
            if(current_link == nullptr) {
                std::string error_msg =  "Error: Link " + target_link_name + " not found.";
                throw std::runtime_error(error_msg);
            }

            // Build kinematic tree from target_link {t} to base {b}
            Eigen::Transform<Scalar, 3, Eigen::Affine> Htb = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
            while(current_link->name != model->base_link->name) {
                if(current_link->joint->type == JointType::REVOLUTE || current_link->joint->type == JointType::PRISMATIC) {
                    Eigen::Matrix<Scalar, 3, 1> axis = current_link->joint->axis;
                    Scalar q_current = q(current_link->joint->q_index);
                    Eigen::Transform<Scalar, 3, Eigen::Affine> H = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
                    if( current_link->joint->type == JointType::REVOLUTE) {
                        // Rotate by q_current around axis
                        H.linear() = Eigen::AngleAxis<Scalar>(q_current, axis).toRotationMatrix();
                    } else {
                        // Translate by q_current along axis
                        H.translation() = axis * q_current;
                    }
                    Htb = Htb*H.inverse();
                }
                // Apply inverse joint transform as we are going back up tree
                Htb = Htb*current_link->joint->parent_transform.inverse();
                // Move up the tree to parent
                current_link = current_link->parent_link;
            }

            // Return transform from base {b} to target {t} : Hbt = Htb.inverse()
            return Htb.inverse();
        }

        /**
         * @brief Computes the transform between source link {s} and target {t} centre of mass (CoM) {c}.
         * @param model The robot model.
         * @param q The joint configuration of the robot.
         * @param source_link_name {s} The link from which the transform is computed.
         * @param target_link_name {t} The link to which the transform is computed.
         * @return The transform between the two links.
         */
        template <typename Scalar>
        Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics_com(std::shared_ptr<RobotModel<Scalar>> model,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q, std::string& source_link_name, std::string& target_link_name) {
            // Assert the configuration vector is valid
            // assert(q.size() == model->n_q); TODO: FIX THIS FOR AUTODIFF

            // Compute forward kinematics from source {b} to target {t}
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hst = forward_kinematics(model, q, source_link_name, target_link_name);

            // Compute forward kinematics from source {s} to CoM {c}
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hsc = Hst * model->get_link(target_link_name)->centre_of_mass;

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
        template <typename Scalar>
        inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1> position(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
                    std::shared_ptr<RobotModel<Scalar>> model,
                    std::string& source_link_name,
                    std::string& target_link_name) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hst = forward_kinematics(model, q, source_link_name, target_link_name);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rTSs(Hst.translation());
            return rTSs;
        }

        /**
         * @brief Computes the orientation of the target link in the source link frame expressed in euler angles. [roll, pitch, yaw]
         * @param model The robot model.
         * @param q The joint configuration of the robot.
         * @param source_link_name {s} The link from which the transform is computed.
         * @param target_link_name {t} The link to which the transform is computed.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        template <typename Scalar>
        inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1> orientation(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
                    std::shared_ptr<RobotModel<Scalar>> model,
                    std::string& source_link_name,
                    std::string& target_link_name) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> Hst = forward_kinematics(model, q, source_link_name, target_link_name);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rpy(Hst.linear().eulerAngles(0, 1, 2));
            return rpy;
        }

        /**
         * @brief Computes translation component of the geometric Jacobian between two links Jv.
         * @param model The robot model.
         * @param q The joint configuration of the robot.
         * @param source_link_name {s} The link from which the transform is computed.
         * @param target_link_name {t} The link to which the transform is computed.
         * @return The translation component of the geometric Jacobian between two links.
         */
        template <typename Scalar>
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jv(
            std::shared_ptr<RobotModel<Scalar>> model,
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
            std::string& source_link_name,
            std::string& target_link_name) {

            // Assert the configuration vector is valid
            assert(q.size() == model->n_q);

            // Cast q and model to autodiff type
            Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> q_real(q); // the input vector q
            std::shared_ptr<RML::RobotModel<autodiff::real>> autodiff_model;
            autodiff_model = model->template cast<autodiff::real>();
            // The output vector F = f(x) evaluated together with Jacobian matrix below
            Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> F;
            // Evaluate the output vector F and the Jacobian matrix dF/dx
            Eigen::MatrixXd Jv = jacobian(position<autodiff::real>, wrt(q_real), at(q_real, autodiff_model, source_link_name, target_link_name), F);
            return Jv;
        }

        // /**
        //  * @brief Computes the centre of mass expressed in source link frame.
        //  *
        //  * @param source_link_id The link from which the centre of mass position is computed.
        //  * @param q The configuration vector of the robot model.
        //  *
        //  * @return The centre of mass position expressed in source link frame.
        //  */
        // Eigen::Matrix<Scalar, 3, 1> centre_of_mass(int source_link_id,
        //                                            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q) const;

}  //namespace RML

#endif
