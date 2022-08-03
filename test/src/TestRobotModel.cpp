#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include <string>

#include "../../src/RobotModel.hpp"

TEST_CASE ("Load a model with inertial information", "[RobotModel]") {
    // Create a robot model
    std::shared_ptr<RML::RobotModel<double>> robot_model;

    // Load the robot model from a URDF file
    robot_model = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

    CHECK(robot_model->name == "compass_gait");

    CHECK(robot_model->links.size() == 9);
    CHECK(robot_model->joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model->links["ground"]->name == "ground");
    CHECK(robot_model->links["ground"]->mass == 0);
    CHECK(robot_model->links["ground"]->mass == 0.0);
    CHECK(robot_model->links["ground"]->parent_link->name == "world");
    CHECK(robot_model->links["ground"]->inertia == Eigen::Matrix<double, 6, 1>::Zero());
    CHECK(robot_model->links["ground"]->centre_of_mass.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model->links["ground"]->id == 3);

    CHECK(robot_model->links["body"]->name == "body");
    CHECK(robot_model->links["body"]->mass == 10.0);
    CHECK(robot_model->links["body"]->parent_link->name == "floating_base_z");
    CHECK(robot_model->links["body"]->inertia == Eigen::Matrix<double, 6, 1>::Zero());
    CHECK(robot_model->links["body"]->centre_of_mass.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model->links["body"]->id == 0);
    CHECK(robot_model->links["body"]->child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model->joints["hip_joint"]->id == 0);
    CHECK(robot_model->joints["hip_joint"]->name == "hip_joint");
    CHECK(robot_model->joints["hip_joint"]->type == RML::JointType::FIXED);
    CHECK(robot_model->joints["hip_joint"]->parent_link_name == "floating_base_z");
    CHECK(robot_model->joints["hip_joint"]->child_link_name == "body");
    CHECK(robot_model->joints["hip_joint"]->axis == Eigen::Matrix<double, 3, 1>::Zero());
    CHECK(robot_model->joints["hip_joint"]->parent_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model->joints["hip_joint"]->child_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model->joints["left_hip_pitch"]->id == 5);
    CHECK(robot_model->joints["left_hip_pitch"]->name == "left_hip_pitch");
    CHECK(robot_model->joints["left_hip_pitch"]->type == RML::JointType::REVOLUTE);
    CHECK(robot_model->joints["left_hip_pitch"]->parent_link_name == "body");
    CHECK(robot_model->joints["left_hip_pitch"]->child_link_name == "left_leg");
    CHECK(robot_model->joints["left_hip_pitch"]->axis == Eigen::Matrix<double, 3, 1>(0, -1, 0));
    CHECK(robot_model->joints["left_hip_pitch"]->parent_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model->joints["left_hip_pitch"]->child_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model->joints["floating_base_x"]->id == 1);
    CHECK(robot_model->joints["floating_base_x"]->name == "floating_base_x");
    CHECK(robot_model->joints["floating_base_x"]->type == RML::JointType::PRISMATIC);
    CHECK(robot_model->joints["floating_base_x"]->parent_link_name == "ground");
    CHECK(robot_model->joints["floating_base_x"]->child_link_name == "floating_base_x");
    CHECK(robot_model->joints["floating_base_x"]->axis == Eigen::Matrix<double, 3, 1>(1, 0, 0));
    CHECK(robot_model->joints["floating_base_x"]->parent_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model->joints["floating_base_x"]->child_transform.matrix() == Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix());

};

TEST_CASE ("Cast a RobotModel from double to float", "[RobotModel]") {
    // Create a robot model
    std::shared_ptr<RML::RobotModel<double>> robot_model_double;

    // Load the robot model from a URDF file
    robot_model_double = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

    // Cast the model to a different type
    std::shared_ptr<RML::RobotModel<float>> robot_model_float = robot_model_double->template cast<float>();

    CHECK(robot_model_float->name == "compass_gait");

    CHECK(robot_model_float->links.size() == 9);
    CHECK(robot_model_float->joints.size() == 8);

    // Check parsed link information
    CHECK(robot_model_float->links["ground"]->name == "ground");
    CHECK(robot_model_float->links["ground"]->mass == 0);
    CHECK(robot_model_float->links["ground"]->mass == 0.0);
    CHECK(robot_model_float->links["ground"]->parent_link->name == "world");
    CHECK(robot_model_float->links["ground"]->inertia == Eigen::Matrix<float, 6, 1>::Zero());
    CHECK(robot_model_float->links["ground"]->centre_of_mass.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float->links["ground"]->id == 3);

    CHECK(robot_model_float->links["body"]->name == "body");
    CHECK(robot_model_float->links["body"]->mass == 10.0);
    CHECK(robot_model_float->links["body"]->parent_link->name == "floating_base_z");
    CHECK(robot_model_float->links["body"]->inertia == Eigen::Matrix<float, 6, 1>::Zero());
    CHECK(robot_model_float->links["body"]->centre_of_mass.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float->links["body"]->id == 0);
    CHECK(robot_model_float->links["body"]->child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(robot_model_float->joints["hip_joint"]->id == 0);
    CHECK(robot_model_float->joints["hip_joint"]->name == "hip_joint");
    CHECK(robot_model_float->joints["hip_joint"]->type == RML::JointType::FIXED);
    CHECK(robot_model_float->joints["hip_joint"]->parent_link_name == "floating_base_z");
    CHECK(robot_model_float->joints["hip_joint"]->child_link_name == "body");
    CHECK(robot_model_float->joints["hip_joint"]->axis == Eigen::Matrix<float, 3, 1>::Zero());
    CHECK(robot_model_float->joints["hip_joint"]->parent_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float->joints["hip_joint"]->child_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(robot_model_float->joints["left_hip_pitch"]->id == 5);
    CHECK(robot_model_float->joints["left_hip_pitch"]->name == "left_hip_pitch");
    CHECK(robot_model_float->joints["left_hip_pitch"]->type == RML::JointType::REVOLUTE);
    CHECK(robot_model_float->joints["left_hip_pitch"]->parent_link_name == "body");
    CHECK(robot_model_float->joints["left_hip_pitch"]->child_link_name == "left_leg");
    CHECK(robot_model_float->joints["left_hip_pitch"]->axis == Eigen::Matrix<float, 3, 1>(0, -1, 0));
    CHECK(robot_model_float->joints["left_hip_pitch"]->parent_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float->joints["left_hip_pitch"]->child_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(robot_model_float->joints["floating_base_x"]->id == 1);
    CHECK(robot_model_float->joints["floating_base_x"]->name == "floating_base_x");
    CHECK(robot_model_float->joints["floating_base_x"]->type == RML::JointType::PRISMATIC);
    CHECK(robot_model_float->joints["floating_base_x"]->parent_link_name == "ground");
    CHECK(robot_model_float->joints["floating_base_x"]->child_link_name == "floating_base_x");
    CHECK(robot_model_float->joints["floating_base_x"]->axis == Eigen::Matrix<float, 3, 1>(1, 0, 0));
    CHECK(robot_model_float->joints["floating_base_x"]->parent_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());
    CHECK(robot_model_float->joints["floating_base_x"]->child_transform.matrix() == Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix());

};

// #include <autodiff/forward/real.hpp>
// #include <autodiff/forward/real/eigen.hpp>
// using namespace autodiff;

// TEST_CASE ("Cast a RobotModel from double to autodiff::real", "[RobotModel]") {
//     // Create a robot model
//     std::shared_ptr<RML::RobotModel<double>> robot_model_double;

//     // Load the robot model from a URDF file
//     robot_model_double = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

//     // Cast the model to a different type
//     std::shared_ptr<RML::RobotModel<autodiff::real>> autodiff_model;
//     autodiff_model = robot_model_double->template cast<autodiff::real>();

//     CHECK(autodiff_model->name == "compass_gait");

//     CHECK(autodiff_model->links.size() == 9);
//     CHECK(autodiff_model->joints.size() == 8);

//     // Check parsed link information
//     CHECK(autodiff_model->links["ground"]->name == "ground");
//     CHECK(autodiff_model->links["ground"]->mass == 0);
//     CHECK(autodiff_model->links["ground"]->mass == 0.0);
//     CHECK(autodiff_model->links["ground"]->parent_link->name == "world");
//     CHECK(autodiff_model->links["ground"]->inertia == Eigen::Matrix<autodiff::real, 6, 1>::Zero());
//     CHECK(autodiff_model->links["ground"]->centre_of_mass.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model->links["ground"]->id == 3);

//     CHECK(autodiff_model->links["body"]->name == "body");
//     CHECK(autodiff_model->links["body"]->mass == 10.0);
//     CHECK(autodiff_model->links["body"]->parent_link->name == "floating_base_z");
//     CHECK(autodiff_model->links["body"]->inertia == Eigen::Matrix<autodiff::real, 6, 1>::Zero());
//     CHECK(autodiff_model->links["body"]->centre_of_mass.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model->links["body"]->id == 0);
//     CHECK(autodiff_model->links["body"]->child_links.size() == 2);

//     // Check parsed fixed joint information
//     CHECK(autodiff_model->joints["hip_joint"]->id == 0);
//     CHECK(autodiff_model->joints["hip_joint"]->name == "hip_joint");
//     CHECK(autodiff_model->joints["hip_joint"]->type == RML::JointType::FIXED);
//     CHECK(autodiff_model->joints["hip_joint"]->parent_link_name == "floating_base_z");
//     CHECK(autodiff_model->joints["hip_joint"]->child_link_name == "body");
//     CHECK(autodiff_model->joints["hip_joint"]->axis == Eigen::Matrix<autodiff::real, 3, 1>::Zero());
//     CHECK(autodiff_model->joints["hip_joint"]->parent_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model->joints["hip_joint"]->child_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());

//     // Check parsed revolute joint information
//     CHECK(autodiff_model->joints["left_hip_pitch"]->id == 5);
//     CHECK(autodiff_model->joints["left_hip_pitch"]->name == "left_hip_pitch");
//     CHECK(autodiff_model->joints["left_hip_pitch"]->type == RML::JointType::REVOLUTE);
//     CHECK(autodiff_model->joints["left_hip_pitch"]->parent_link_name == "body");
//     CHECK(autodiff_model->joints["left_hip_pitch"]->child_link_name == "left_leg");
//     CHECK(autodiff_model->joints["left_hip_pitch"]->axis == Eigen::Matrix<autodiff::real, 3, 1>(0, -1, 0));
//     CHECK(autodiff_model->joints["left_hip_pitch"]->parent_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model->joints["left_hip_pitch"]->child_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());

//     // Check parsed prismatic joint information
//     CHECK(autodiff_model->joints["floating_base_x"]->id == 1);
//     CHECK(autodiff_model->joints["floating_base_x"]->name == "floating_base_x");
//     CHECK(autodiff_model->joints["floating_base_x"]->type == RML::JointType::PRISMATIC);
//     CHECK(autodiff_model->joints["floating_base_x"]->parent_link_name == "ground");
//     CHECK(autodiff_model->joints["floating_base_x"]->child_link_name == "floating_base_x");
//     CHECK(autodiff_model->joints["floating_base_x"]->axis == Eigen::Matrix<autodiff::real, 3, 1>(1, 0, 0));
//     CHECK(autodiff_model->joints["floating_base_x"]->parent_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());
//     CHECK(autodiff_model->joints["floating_base_x"]->child_transform.matrix() == Eigen::Transform<autodiff::real, 3, Eigen::Affine>::Identity().matrix());

// };

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

TEST_CASE ("Cast a RobotModel from double to autodiff::dual", "[RobotModel]") {
    // Create a robot model
    std::shared_ptr<RML::RobotModel<double>> robot_model_double;

    // Load the robot model from a URDF file
    robot_model_double = RML::RobotModel<double>::from_urdf("data/urdfs/simple.urdf");

    // Cast the model to a different type
    std::shared_ptr<RML::RobotModel<autodiff::dual>> autodiff_model;
    autodiff_model = robot_model_double->template cast<autodiff::dual>();

    CHECK(autodiff_model->name == "compass_gait");

    CHECK(autodiff_model->links.size() == 9);
    CHECK(autodiff_model->joints.size() == 8);

    // Check parsed link information
    CHECK(autodiff_model->links["ground"]->name == "ground");
    CHECK(autodiff_model->links["ground"]->mass == 0);
    CHECK(autodiff_model->links["ground"]->mass == 0.0);
    CHECK(autodiff_model->links["ground"]->parent_link->name == "world");
    CHECK(autodiff_model->links["ground"]->inertia == Eigen::Matrix<autodiff::dual, 6, 1>::Zero());
    CHECK(autodiff_model->links["ground"]->centre_of_mass.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model->links["ground"]->id == 3);

    CHECK(autodiff_model->links["body"]->name == "body");
    CHECK(autodiff_model->links["body"]->mass == 10.0);
    CHECK(autodiff_model->links["body"]->parent_link->name == "floating_base_z");
    CHECK(autodiff_model->links["body"]->inertia == Eigen::Matrix<autodiff::dual, 6, 1>::Zero());
    CHECK(autodiff_model->links["body"]->centre_of_mass.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model->links["body"]->id == 0);
    CHECK(autodiff_model->links["body"]->child_links.size() == 2);

    // Check parsed fixed joint information
    CHECK(autodiff_model->joints["hip_joint"]->id == 0);
    CHECK(autodiff_model->joints["hip_joint"]->name == "hip_joint");
    CHECK(autodiff_model->joints["hip_joint"]->type == RML::JointType::FIXED);
    CHECK(autodiff_model->joints["hip_joint"]->parent_link_name == "floating_base_z");
    CHECK(autodiff_model->joints["hip_joint"]->child_link_name == "body");
    CHECK(autodiff_model->joints["hip_joint"]->axis == Eigen::Matrix<autodiff::dual, 3, 1>::Zero());
    CHECK(autodiff_model->joints["hip_joint"]->parent_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model->joints["hip_joint"]->child_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed revolute joint information
    CHECK(autodiff_model->joints["left_hip_pitch"]->id == 5);
    CHECK(autodiff_model->joints["left_hip_pitch"]->name == "left_hip_pitch");
    CHECK(autodiff_model->joints["left_hip_pitch"]->type == RML::JointType::REVOLUTE);
    CHECK(autodiff_model->joints["left_hip_pitch"]->parent_link_name == "body");
    CHECK(autodiff_model->joints["left_hip_pitch"]->child_link_name == "left_leg");
    CHECK(autodiff_model->joints["left_hip_pitch"]->axis == Eigen::Matrix<autodiff::dual, 3, 1>(0, -1, 0));
    CHECK(autodiff_model->joints["left_hip_pitch"]->parent_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model->joints["left_hip_pitch"]->child_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());

    // Check parsed prismatic joint information
    CHECK(autodiff_model->joints["floating_base_x"]->id == 1);
    CHECK(autodiff_model->joints["floating_base_x"]->name == "floating_base_x");
    CHECK(autodiff_model->joints["floating_base_x"]->type == RML::JointType::PRISMATIC);
    CHECK(autodiff_model->joints["floating_base_x"]->parent_link_name == "ground");
    CHECK(autodiff_model->joints["floating_base_x"]->child_link_name == "floating_base_x");
    CHECK(autodiff_model->joints["floating_base_x"]->axis == Eigen::Matrix<autodiff::dual, 3, 1>(1, 0, 0));
    CHECK(autodiff_model->joints["floating_base_x"]->parent_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());
    CHECK(autodiff_model->joints["floating_base_x"]->child_transform.matrix() == Eigen::Transform<autodiff::dual, 3, Eigen::Affine>::Identity().matrix());

};
