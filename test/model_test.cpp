#define CATCH_CONFIG_MAIN
#include <string>

#include "../include/parser.hpp"
#include "catch2/catch.hpp"

using namespace tinyrobotics;

TEST_CASE("Load a model with inertial information", "[Model]") {
    // Create a robot model
    auto robot_model = importURDF<double, 4>("data/urdfs/simple.urdf");

    CHECK(robot_model.name == "compass_gait");

    CHECK(robot_model.links.size() == 9);

    // Check parsed link information
    CHECK(robot_model.getLink("ground").name == "ground");
    CHECK(robot_model.getLink("ground").mass == 0);
    CHECK(robot_model.getParentLink("ground").name == "world");
    CHECK(robot_model.getLink("ground").inertia == Eigen::Matrix<double, 3, 3>::Zero());
    CHECK(robot_model.getLink("ground").centreOfMass.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.getLink("body").name == "body");
    CHECK(robot_model.getLink("body").mass == 10.0);
    CHECK(robot_model.getParentLink("body").name == "floating_base_z");
    CHECK(robot_model.getLink("body").inertia == Eigen::Matrix<double, 3, 3>::Zero());
    CHECK(robot_model.getLink("body").centreOfMass.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.getLink("body").child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model.getJoint("hip_joint").name == "hip_joint");
    CHECK(robot_model.getJoint("hip_joint").type == JointType::FIXED);
    CHECK(robot_model.getJoint("hip_joint").parent_link_name == "floating_base_z");
    CHECK(robot_model.getJoint("hip_joint").child_link_name == "body");
    CHECK(robot_model.getJoint("hip_joint").axis == Eigen::Matrix<double, 3, 1>::Zero());
    CHECK(robot_model.getJoint("hip_joint").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.getJoint("hip_joint").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model.getJoint("left_hip_pitch").name == "left_hip_pitch");
    CHECK(robot_model.getJoint("left_hip_pitch").type == JointType::REVOLUTE);
    CHECK(robot_model.getJoint("left_hip_pitch").parent_link_name == "body");
    CHECK(robot_model.getJoint("left_hip_pitch").child_link_name == "left_leg");
    CHECK(robot_model.getJoint("left_hip_pitch").axis == Eigen::Matrix<double, 3, 1>(0, -1, 0));
    CHECK(robot_model.getJoint("left_hip_pitch").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.getJoint("left_hip_pitch").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model.getJoint("floating_base_x").name == "floating_base_x");
    CHECK(robot_model.getJoint("floating_base_x").type == JointType::PRISMATIC);
    CHECK(robot_model.getJoint("floating_base_x").parent_link_name == "ground");
    CHECK(robot_model.getJoint("floating_base_x").child_link_name == "floating_base_x");
    CHECK(robot_model.getJoint("floating_base_x").axis == Eigen::Matrix<double, 3, 1>(1, 0, 0));
    CHECK(robot_model.getJoint("floating_base_x").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.getJoint("floating_base_x").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
};

TEST_CASE("Cast a Model from double to float", "[Model]") {
    // Create a robot model
    auto robot_model_double = tinyrobotics::importURDF<double, 4>("data/urdfs/simple.urdf");

    // Cast the model to a different type
    auto robot_model_float = robot_model_double.template cast<float>();

    CHECK(robot_model_float.name == "compass_gait");

    CHECK(robot_model_float.links.size() == 9);

    // Check parsed link information
    CHECK(robot_model_float.getLink("ground").name == "ground");
    CHECK(robot_model_float.getLink("ground").mass == 0);
    CHECK(robot_model_float.getParentLink("ground").name == "world");
    CHECK(robot_model_float.getLink("ground").inertia == Eigen::Matrix<float, 3, 3>::Zero());
    CHECK(robot_model_float.getLink("ground").centreOfMass.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    CHECK(robot_model_float.getLink("body").name == "body");
    CHECK(robot_model_float.getLink("body").mass == 10.0);
    CHECK(robot_model_float.getParentLink("body").name == "floating_base_z");
    CHECK(robot_model_float.getLink("body").inertia == Eigen::Matrix<float, 3, 3>::Zero());
    CHECK(robot_model_float.getLink("body").centreOfMass.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.getLink("body").child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model_float.getJoint("hip_joint").name == "hip_joint");
    CHECK(robot_model_float.getJoint("hip_joint").type == tinyrobotics::JointType::FIXED);
    CHECK(robot_model_float.getJoint("hip_joint").parent_link_name == "floating_base_z");
    CHECK(robot_model_float.getJoint("hip_joint").child_link_name == "body");
    CHECK(robot_model_float.getJoint("hip_joint").axis == Eigen::Matrix<float, 3, 1>::Zero());
    CHECK(robot_model_float.getJoint("hip_joint").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.getJoint("hip_joint").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model_float.getJoint("left_hip_pitch").name == "left_hip_pitch");
    CHECK(robot_model_float.getJoint("left_hip_pitch").type == tinyrobotics::JointType::REVOLUTE);
    CHECK(robot_model_float.getJoint("left_hip_pitch").parent_link_name == "body");
    CHECK(robot_model_float.getJoint("left_hip_pitch").child_link_name == "left_leg");
    CHECK(robot_model_float.getJoint("left_hip_pitch").axis == Eigen::Matrix<float, 3, 1>(0, -1, 0));
    CHECK(robot_model_float.getJoint("left_hip_pitch").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.getJoint("left_hip_pitch").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model_float.getJoint("floating_base_x").name == "floating_base_x");
    CHECK(robot_model_float.getJoint("floating_base_x").type == tinyrobotics::JointType::PRISMATIC);
    CHECK(robot_model_float.getJoint("floating_base_x").parent_link_name == "ground");
    CHECK(robot_model_float.getJoint("floating_base_x").child_link_name == "floating_base_x");
    CHECK(robot_model_float.getJoint("floating_base_x").axis == Eigen::Matrix<float, 3, 1>(1, 0, 0));
    CHECK(robot_model_float.getJoint("floating_base_x").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.getJoint("floating_base_x").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
};