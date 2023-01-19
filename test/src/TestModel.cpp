#define CATCH_CONFIG_MAIN
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <string>

#include "../../include/UrdfParser.hpp"
#include "catch2/catch.hpp"

TEST_CASE("Load a model with inertial information", "[Model]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<double, 4>("data/urdfs/simple.urdf");

    CHECK(robot_model.name == "compass_gait");

    CHECK(robot_model.links.size() == 9);
    CHECK(robot_model.joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model.get_link("ground").name == "ground");
    CHECK(robot_model.get_link("ground").mass == 0);
    CHECK(robot_model.get_parent_link("ground").name == "world");
    CHECK(robot_model.get_link("ground").inertia == Eigen::Matrix<double, 3, 3>::Zero());
    CHECK(robot_model.get_link("ground").centre_of_mass.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.get_link("body").name == "body");
    CHECK(robot_model.get_link("body").mass == 10.0);
    CHECK(robot_model.get_parent_link("body").name == "floating_base_z");
    CHECK(robot_model.get_link("body").inertia == Eigen::Matrix<double, 3, 3>::Zero());
    CHECK(robot_model.get_link("body").centre_of_mass.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.get_link("body").child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model.get_joint("hip_joint").name == "hip_joint");
    CHECK(robot_model.get_joint("hip_joint").type == RML::JointType::FIXED);
    CHECK(robot_model.get_joint("hip_joint").parent_link_name == "floating_base_z");
    CHECK(robot_model.get_joint("hip_joint").child_link_name == "body");
    CHECK(robot_model.get_joint("hip_joint").axis == Eigen::Matrix<double, 3, 1>::Zero());
    CHECK(robot_model.get_joint("hip_joint").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.get_joint("hip_joint").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model.get_joint("left_hip_pitch").name == "left_hip_pitch");
    CHECK(robot_model.get_joint("left_hip_pitch").type == RML::JointType::REVOLUTE);
    CHECK(robot_model.get_joint("left_hip_pitch").parent_link_name == "body");
    CHECK(robot_model.get_joint("left_hip_pitch").child_link_name == "left_leg");
    CHECK(robot_model.get_joint("left_hip_pitch").axis == Eigen::Matrix<double, 3, 1>(0, -1, 0));
    CHECK(robot_model.get_joint("left_hip_pitch").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.get_joint("left_hip_pitch").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model.get_joint("floating_base_x").name == "floating_base_x");
    CHECK(robot_model.get_joint("floating_base_x").type == RML::JointType::PRISMATIC);
    CHECK(robot_model.get_joint("floating_base_x").parent_link_name == "ground");
    CHECK(robot_model.get_joint("floating_base_x").child_link_name == "floating_base_x");
    CHECK(robot_model.get_joint("floating_base_x").axis == Eigen::Matrix<double, 3, 1>(1, 0, 0));
    CHECK(robot_model.get_joint("floating_base_x").parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model.get_joint("floating_base_x").child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Isometry>::Identity().matrix());
};

TEST_CASE("Create a Model of autodiff type", "[Model]") {
    // Create a robot model
    auto robot_model = RML::model_from_urdf<autodiff::real, 4>("data/urdfs/simple.urdf");
    CHECK(robot_model.name == "compass_gait");
}

TEST_CASE("Cast a Model from double to float", "[Model]") {
    // Create a robot model
    auto robot_model_double = RML::model_from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Cast the model to a different type
    auto robot_model_float = robot_model_double.template cast<float>();

    CHECK(robot_model_float.name == "compass_gait");

    CHECK(robot_model_float.links.size() == 9);
    CHECK(robot_model_float.joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model_float.get_link("ground").name == "ground");
    CHECK(robot_model_float.get_link("ground").mass == 0);
    CHECK(robot_model_float.get_parent_link("ground").name == "world");
    CHECK(robot_model_float.get_link("ground").inertia == Eigen::Matrix<float, 3, 3>::Zero());
    CHECK(robot_model_float.get_link("ground").centre_of_mass.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    CHECK(robot_model_float.get_link("body").name == "body");
    CHECK(robot_model_float.get_link("body").mass == 10.0);
    CHECK(robot_model_float.get_parent_link("body").name == "floating_base_z");
    CHECK(robot_model_float.get_link("body").inertia == Eigen::Matrix<float, 3, 3>::Zero());
    CHECK(robot_model_float.get_link("body").centre_of_mass.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.get_link("body").child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model_float.get_joint("hip_joint").name == "hip_joint");
    CHECK(robot_model_float.get_joint("hip_joint").type == RML::JointType::FIXED);
    CHECK(robot_model_float.get_joint("hip_joint").parent_link_name == "floating_base_z");
    CHECK(robot_model_float.get_joint("hip_joint").child_link_name == "body");
    CHECK(robot_model_float.get_joint("hip_joint").axis == Eigen::Matrix<float, 3, 1>::Zero());
    CHECK(robot_model_float.get_joint("hip_joint").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.get_joint("hip_joint").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model_float.get_joint("left_hip_pitch").name == "left_hip_pitch");
    CHECK(robot_model_float.get_joint("left_hip_pitch").type == RML::JointType::REVOLUTE);
    CHECK(robot_model_float.get_joint("left_hip_pitch").parent_link_name == "body");
    CHECK(robot_model_float.get_joint("left_hip_pitch").child_link_name == "left_leg");
    CHECK(robot_model_float.get_joint("left_hip_pitch").axis == Eigen::Matrix<float, 3, 1>(0, -1, 0));
    CHECK(robot_model_float.get_joint("left_hip_pitch").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.get_joint("left_hip_pitch").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model_float.get_joint("floating_base_x").name == "floating_base_x");
    CHECK(robot_model_float.get_joint("floating_base_x").type == RML::JointType::PRISMATIC);
    CHECK(robot_model_float.get_joint("floating_base_x").parent_link_name == "ground");
    CHECK(robot_model_float.get_joint("floating_base_x").child_link_name == "floating_base_x");
    CHECK(robot_model_float.get_joint("floating_base_x").axis == Eigen::Matrix<float, 3, 1>(1, 0, 0));
    CHECK(robot_model_float.get_joint("floating_base_x").parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(robot_model_float.get_joint("floating_base_x").child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Isometry>::Identity().matrix());
};


TEST_CASE("Cast a Model from double to autodiff::real", "[Model]") {
    // Load the robot model from a URDF file
    auto robot_model_double = RML::model_from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Cast the model to a different type
    auto autodiff_model = robot_model_double.template cast<autodiff::real>();

    CHECK(autodiff_model.name == "compass_gait");

    CHECK(autodiff_model.links.size() == 9);
    CHECK(autodiff_model.joints.size() == 8);

    // Check parsed link information
    CHECK(autodiff_model.get_link("ground").name == "ground");
    CHECK(autodiff_model.get_link("ground").mass == 0);
    CHECK(autodiff_model.get_link("ground").mass == 0.0);
    CHECK(autodiff_model.get_parent_link("ground").name == "world");
    CHECK(autodiff_model.get_link("ground").inertia == Eigen::Matrix<autodiff::real, 3, 3>::Zero());
    CHECK(autodiff_model.get_link("ground").centre_of_mass.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());

    CHECK(autodiff_model.get_link("body").name == "body");
    CHECK(autodiff_model.get_link("body").mass == 10.0);
    CHECK(autodiff_model.get_parent_link("body").name == "floating_base_z");
    CHECK(autodiff_model.get_link("body").inertia == Eigen::Matrix<autodiff::real, 3, 3>::Zero());
    CHECK(autodiff_model.get_link("body").centre_of_mass.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(autodiff_model.get_link("body").child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(autodiff_model.get_joint("hip_joint").name == "hip_joint");
    CHECK(autodiff_model.get_joint("hip_joint").type == RML::JointType::FIXED);
    CHECK(autodiff_model.get_joint("hip_joint").parent_link_name == "floating_base_z");
    CHECK(autodiff_model.get_joint("hip_joint").child_link_name == "body");
    CHECK(autodiff_model.get_joint("hip_joint").axis == Eigen::Matrix<autodiff::real, 3, 1>::Zero());
    CHECK(autodiff_model.get_joint("hip_joint").parent_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(autodiff_model.get_joint("hip_joint").child_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(autodiff_model.get_joint("left_hip_pitch").name == "left_hip_pitch");
    CHECK(autodiff_model.get_joint("left_hip_pitch").type == RML::JointType::REVOLUTE);
    CHECK(autodiff_model.get_joint("left_hip_pitch").parent_link_name == "body");
    CHECK(autodiff_model.get_joint("left_hip_pitch").child_link_name == "left_leg");
    CHECK(autodiff_model.get_joint("left_hip_pitch").axis == Eigen::Matrix<autodiff::real, 3, 1>(0, -1, 0));
    CHECK(autodiff_model.get_joint("left_hip_pitch").parent_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(autodiff_model.get_joint("left_hip_pitch").child_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(autodiff_model.get_joint("floating_base_x").name == "floating_base_x");
    CHECK(autodiff_model.get_joint("floating_base_x").type == RML::JointType::PRISMATIC);
    CHECK(autodiff_model.get_joint("floating_base_x").parent_link_name == "ground");
    CHECK(autodiff_model.get_joint("floating_base_x").child_link_name == "floating_base_x");
    CHECK(autodiff_model.get_joint("floating_base_x").axis == Eigen::Matrix<autodiff::real, 3, 1>(1, 0, 0));
    CHECK(autodiff_model.get_joint("floating_base_x").parent_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());
    CHECK(autodiff_model.get_joint("floating_base_x").child_transform.matrix()
          == Eigen::Transform<autodiff::real, 3, Eigen::Isometry>::Identity().matrix());
};
