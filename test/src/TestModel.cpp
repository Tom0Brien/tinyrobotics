#define CATCH_CONFIG_MAIN
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <string>

#include "../../src/Model.hpp"
#include "catch2/catch.hpp"

TEST_CASE("Load a model with inertial information", "[Model]") {
    // Create a robot model
    auto robot_model = RML::from_urdf<double, 4>("data/urdfs/simple.urdf");

    CHECK(robot_model.name == "compass_gait");

    CHECK(robot_model.links.size() == 9);
    CHECK(robot_model.joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model.get_link("ground")->name == "ground");
    CHECK(robot_model.get_link("ground")->mass == 0);
    CHECK(robot_model.get_link("ground")->mass == 0.0);
    CHECK(robot_model.get_link("ground")->parent_link->name == "world");
    CHECK(robot_model.get_link("ground")->inertia == Eigen::Matrix<double, 6, 1>::Zero());
    CHECK(robot_model.get_link("ground")->centre_of_mass.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(robot_model.get_link("ground")->id == 3);

    CHECK(robot_model.get_link("body")->name == "body");
    CHECK(robot_model.get_link("body")->mass == 10.0);
    CHECK(robot_model.get_link("body")->parent_link->name == "floating_base_z");
    CHECK(robot_model.get_link("body")->inertia == Eigen::Matrix<double, 6, 1>::Zero());
    CHECK(robot_model.get_link("body")->centre_of_mass.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(robot_model.get_link("body")->id == 0);
    CHECK(robot_model.get_link("body")->child_links.size() == 2);

    // Check parsed fixed joint information
    // CHECK(robot_model.get_joint("hip_joint")->id == 0);
    CHECK(robot_model.get_joint("hip_joint")->name == "hip_joint");
    CHECK(robot_model.get_joint("hip_joint")->type == RML::JointType::FIXED);
    CHECK(robot_model.get_joint("hip_joint")->parent_link_name == "floating_base_z");
    CHECK(robot_model.get_joint("hip_joint")->child_link_name == "body");
    CHECK(robot_model.get_joint("hip_joint")->axis == Eigen::Matrix<double, 3, 1>::Zero());
    CHECK(robot_model.get_joint("hip_joint")->parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model.get_joint("hip_joint")->child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    // CHECK(robot_model.get_joint("left_hip_pitch")->id == 5);
    CHECK(robot_model.get_joint("left_hip_pitch")->name == "left_hip_pitch");
    CHECK(robot_model.get_joint("left_hip_pitch")->type == RML::JointType::REVOLUTE);
    CHECK(robot_model.get_joint("left_hip_pitch")->parent_link_name == "body");
    CHECK(robot_model.get_joint("left_hip_pitch")->child_link_name == "left_leg");
    CHECK(robot_model.get_joint("left_hip_pitch")->axis == Eigen::Matrix<double, 3, 1>(0, -1, 0));
    CHECK(robot_model.get_joint("left_hip_pitch")->parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model.get_joint("left_hip_pitch")->child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    // CHECK(robot_model.get_joint("floating_base_x")->id == 1);
    CHECK(robot_model.get_joint("floating_base_x")->name == "floating_base_x");
    CHECK(robot_model.get_joint("floating_base_x")->type == RML::JointType::PRISMATIC);
    CHECK(robot_model.get_joint("floating_base_x")->parent_link_name == "ground");
    CHECK(robot_model.get_joint("floating_base_x")->child_link_name == "floating_base_x");
    CHECK(robot_model.get_joint("floating_base_x")->axis == Eigen::Matrix<double, 3, 1>(1, 0, 0));
    CHECK(robot_model.get_joint("floating_base_x")->parent_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model.get_joint("floating_base_x")->child_transform.matrix()
          == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
};

TEST_CASE("Create a Model of autodiff type", "[Model]") {
    // Create a robot model
    auto robot_model = RML::from_urdf<double, 4>("data/urdfs/simple.urdf");
    CHECK(robot_model.name == "compass_gait");
}

TEST_CASE("Cast a Model from double to float", "[Model]") {
    // Create a robot model
    auto robot_model_double = RML::from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Cast the model to a different type
    RML::Model<float, 4> robot_model_float;
    robot_model_float = robot_model_double.template cast<float, 4>();

    CHECK(robot_model_float.name == "compass_gait");

    CHECK(robot_model_float.links.size() == 9);
    CHECK(robot_model_float.joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model_float.get_link("ground")->name == "ground");
    CHECK(robot_model_float.get_link("ground")->mass == 0);
    CHECK(robot_model_float.get_link("ground")->mass == 0.0);
    CHECK(robot_model_float.get_link("ground")->parent_link->name == "world");
    CHECK(robot_model_float.get_link("ground")->inertia == Eigen::Matrix<float, 6, 1>::Zero());
    CHECK(robot_model_float.get_link("ground")->centre_of_mass.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(robot_model_float.get_link("ground")->id == 3);

    CHECK(robot_model_float.get_link("body")->name == "body");
    CHECK(robot_model_float.get_link("body")->mass == 10.0);
    CHECK(robot_model_float.get_link("body")->parent_link->name == "floating_base_z");
    CHECK(robot_model_float.get_link("body")->inertia == Eigen::Matrix<float, 6, 1>::Zero());
    CHECK(robot_model_float.get_link("body")->centre_of_mass.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(robot_model_float.get_link("body")->id == 0);
    CHECK(robot_model_float.get_link("body")->child_links.size() == 2);

    // Check parsed fixed joint information
    // CHECK(robot_model_float.get_joint("hip_joint")->id == 0);
    CHECK(robot_model_float.get_joint("hip_joint")->name == "hip_joint");
    CHECK(robot_model_float.get_joint("hip_joint")->type == RML::JointType::FIXED);
    CHECK(robot_model_float.get_joint("hip_joint")->parent_link_name == "floating_base_z");
    CHECK(robot_model_float.get_joint("hip_joint")->child_link_name == "body");
    CHECK(robot_model_float.get_joint("hip_joint")->axis == Eigen::Matrix<float, 3, 1>::Zero());
    CHECK(robot_model_float.get_joint("hip_joint")->parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float.get_joint("hip_joint")->child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    // CHECK(robot_model_float.get_joint("left_hip_pitch")->id == 5);
    CHECK(robot_model_float.get_joint("left_hip_pitch")->name == "left_hip_pitch");
    CHECK(robot_model_float.get_joint("left_hip_pitch")->type == RML::JointType::REVOLUTE);
    CHECK(robot_model_float.get_joint("left_hip_pitch")->parent_link_name == "body");
    CHECK(robot_model_float.get_joint("left_hip_pitch")->child_link_name == "left_leg");
    CHECK(robot_model_float.get_joint("left_hip_pitch")->axis == Eigen::Matrix<float, 3, 1>(0, -1, 0));
    CHECK(robot_model_float.get_joint("left_hip_pitch")->parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float.get_joint("left_hip_pitch")->child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    // CHECK(robot_model_float.get_joint("floating_base_x")->id == 1);
    CHECK(robot_model_float.get_joint("floating_base_x")->name == "floating_base_x");
    CHECK(robot_model_float.get_joint("floating_base_x")->type == RML::JointType::PRISMATIC);
    CHECK(robot_model_float.get_joint("floating_base_x")->parent_link_name == "ground");
    CHECK(robot_model_float.get_joint("floating_base_x")->child_link_name == "floating_base_x");
    CHECK(robot_model_float.get_joint("floating_base_x")->axis == Eigen::Matrix<float, 3, 1>(1, 0, 0));
    CHECK(robot_model_float.get_joint("floating_base_x")->parent_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float.get_joint("floating_base_x")->child_transform.matrix()
          == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
};

// #include <autodiff/forward/real.hpp>
// #include <autodiff/forward/real/eigen.hpp>
// using namespace autodiff;

// TEST_CASE ("Cast a Model from double to autodiff::real", "[Model]") {
//     // Create a robot model
//     RML::Model<double>> robot_modl_double;

//     // Load the robot model from a URDF file
//     robot_model_double = RML::Model<double>::from_urdf("data/urdfs/simple.urdf");

//     // Cast the model to a different type
//     RML::Model<autodiff::real>> atodiff_model;
//     autodiff_model = robot_model_double->template cast<autodiff::real>();

//     CHECK(autodiff_model.name == "compass_gait");

//     CHECK(autodiff_model.links.size() == 9);
//     CHECK(autodiff_model.joints.size() == 8);

//     // Check parsed link information
//     CHECK(autodiff_model.get_link("ground")->name == "ground");
//     CHECK(autodiff_model.get_link("ground")->mass == 0);
//     CHECK(autodiff_model.get_link("ground")->mass == 0.0);
//     CHECK(autodiff_model.get_link("ground")->parent_link->name == "world");
//     CHECK(autodiff_model.get_link("ground")->inertia == Eigen::Matrix<autodiff::real, 6, 1>::Zero());
//     CHECK(autodiff_model.get_link("ground")->centre_of_mass.matrix() == Eigen::Transform<autodiff::real, 3,
//     Eigen::Affine>::Identity().matrix());
// CHECK(autodiff_model.get_link("ground")->id == 3);

//     CHECK(autodiff_model.get_link("body")->name == "body");
//     CHECK(autodiff_model.get_link("body")->mass == 10.0);
//     CHECK(autodiff_model.get_link("body")->parent_link->name == "floating_base_z");
//     CHECK(autodiff_model.get_link("body")->inertia == Eigen::Matrix<autodiff::real, 6, 1>::Zero());
//     CHECK(autodiff_model.get_link("body")->centre_of_mass.matrix() == Eigen::Transform<autodiff::real, 3,
//     Eigen::Affine>::Identity().matrix());
// CHECK(autodiff_model.get_link("body")->id == 0);
//     CHECK(autodiff_model.get_link("body")->child_links.size() == 2);

//     // Check parsed fixed joint information
// CHECK(autodiff_model.get_joint("hip_joint")->id == 0);
//     CHECK(autodiff_model.get_joint("hip_joint")->name == "hip_joint");
//     CHECK(autodiff_model.get_joint("hip_joint")->type == RML::JointType::FIXED);
//     CHECK(autodiff_model.get_joint("hip_joint")->parent_link_name == "floating_base_z");
//     CHECK(autodiff_model.get_joint("hip_joint")->child_link_name == "body");
//     CHECK(autodiff_model.get_joint("hip_joint")->axis == Eigen::Matrix<autodiff::real, 3, 1>::Zero());
//     CHECK(autodiff_model.get_joint("hip_joint")->parent_transform.matrix() == Eigen::Transform<autodiff::real,
//     3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model.get_joint("hip_joint")->child_transform.matrix() == Eigen::Transform<autodiff::real, 3,
//     Eigen::Affine>::Identity().matrix());

//     // Check parsed revolute joint information
// CHECK(autodiff_model.get_joint("left_hip_pitch")->id == 5);
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->name == "left_hip_pitch");
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->type == RML::JointType::REVOLUTE);
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->parent_link_name == "body");
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->child_link_name == "left_leg");
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->axis == Eigen::Matrix<autodiff::real, 3, 1>(0, -1, 0));
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->parent_transform.matrix() ==
//     Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model.get_joint("left_hip_pitch")->child_transform.matrix() ==
//     Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());

//     // Check parsed prismatic joint information
// CHECK(autodiff_model.get_joint("floating_base_x")->id == 1);
//     CHECK(autodiff_model.get_joint("floating_base_x")->name == "floating_base_x");
//     CHECK(autodiff_model.get_joint("floating_base_x")->type == RML::JointType::PRISMATIC);
//     CHECK(autodiff_model.get_joint("floating_base_x")->parent_link_name == "ground");
//     CHECK(autodiff_model.get_joint("floating_base_x")->child_link_name == "floating_base_x");
//     CHECK(autodiff_model.get_joint("floating_base_x")->axis == Eigen::Matrix<autodiff::real, 3, 1>(1, 0, 0));
//     CHECK(autodiff_model.get_joint("floating_base_x")->parent_transform.matrix() ==
//     Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model.get_joint("floating_base_x")->child_transform.matrix() ==
//     Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());

// };

TEST_CASE("Cast a Model from double to autodiff::dual", "[Model]") {
    // Create a robot model
    auto robot_model_double = RML::from_urdf<double, 4>("data/urdfs/simple.urdf");

    // Cast the model to a different type
    RML::Model<autodiff::dual, 4> autodiff_model;
    autodiff_model = robot_model_double.template cast<autodiff::dual, 4>();

    CHECK(autodiff_model.name == "compass_gait");

    CHECK(autodiff_model.links.size() == 9);
    CHECK(autodiff_model.joints.size() == 8);

    // Check parsed link information
    CHECK(autodiff_model.get_link("ground")->name == "ground");
    CHECK(autodiff_model.get_link("ground")->mass == 0);
    CHECK(autodiff_model.get_link("ground")->mass == 0.0);
    CHECK(autodiff_model.get_link("ground")->parent_link->name == "world");
    CHECK(autodiff_model.get_link("ground")->inertia == Eigen::Matrix<autodiff::dual, 6, 1>::Zero());
    CHECK(autodiff_model.get_link("ground")->centre_of_mass.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(autodiff_model.get_link("ground")->id == 3);

    CHECK(autodiff_model.get_link("body")->name == "body");
    CHECK(autodiff_model.get_link("body")->mass == 10.0);
    CHECK(autodiff_model.get_link("body")->parent_link->name == "floating_base_z");
    CHECK(autodiff_model.get_link("body")->inertia == Eigen::Matrix<autodiff::dual, 6, 1>::Zero());
    CHECK(autodiff_model.get_link("body")->centre_of_mass.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    // CHECK(autodiff_model.get_link("body")->id == 0);
    CHECK(autodiff_model.get_link("body")->child_links.size() == 2);

    // Check parsed fixed joint information
    // CHECK(autodiff_model.get_joint("hip_joint")->id == 0);
    CHECK(autodiff_model.get_joint("hip_joint")->name == "hip_joint");
    CHECK(autodiff_model.get_joint("hip_joint")->type == RML::JointType::FIXED);
    CHECK(autodiff_model.get_joint("hip_joint")->parent_link_name == "floating_base_z");
    CHECK(autodiff_model.get_joint("hip_joint")->child_link_name == "body");
    CHECK(autodiff_model.get_joint("hip_joint")->axis == Eigen::Matrix<autodiff::dual, 3, 1>::Zero());
    CHECK(autodiff_model.get_joint("hip_joint")->parent_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model.get_joint("hip_joint")->child_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    // CHECK(autodiff_model.get_joint("left_hip_pitch")->id == 5);
    CHECK(autodiff_model.get_joint("left_hip_pitch")->name == "left_hip_pitch");
    CHECK(autodiff_model.get_joint("left_hip_pitch")->type == RML::JointType::REVOLUTE);
    CHECK(autodiff_model.get_joint("left_hip_pitch")->parent_link_name == "body");
    CHECK(autodiff_model.get_joint("left_hip_pitch")->child_link_name == "left_leg");
    CHECK(autodiff_model.get_joint("left_hip_pitch")->axis == Eigen::Matrix<autodiff::dual, 3, 1>(0, -1, 0));
    CHECK(autodiff_model.get_joint("left_hip_pitch")->parent_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model.get_joint("left_hip_pitch")->child_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    // CHECK(autodiff_model.get_joint("floating_base_x")->id == 1);
    CHECK(autodiff_model.get_joint("floating_base_x")->name == "floating_base_x");
    CHECK(autodiff_model.get_joint("floating_base_x")->type == RML::JointType::PRISMATIC);
    CHECK(autodiff_model.get_joint("floating_base_x")->parent_link_name == "ground");
    CHECK(autodiff_model.get_joint("floating_base_x")->child_link_name == "floating_base_x");
    CHECK(autodiff_model.get_joint("floating_base_x")->axis == Eigen::Matrix<autodiff::dual, 3, 1>(1, 0, 0));
    CHECK(autodiff_model.get_joint("floating_base_x")->parent_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model.get_joint("floating_base_x")->child_transform.matrix()
          == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
};
