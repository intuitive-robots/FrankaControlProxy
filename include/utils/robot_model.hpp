#pragma once
#include <Eigen/Dense>
#include <array>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <vector>

#include "utils/Pose.h"

// #include "utils/robot_utils.hpp"

using JointPosition = Eigen::Matrix<double, 7, 1>;
using JointVelocity = Eigen::Matrix<double, 7, 1>;
using JointAcceleration = Eigen::Matrix<double, 7, 1>;
using JointTorque = Eigen::Matrix<double, 7, 1>;
using JocobianMatrix = Eigen::Matrix<double, 6, 7>;
constexpr int NUM_DOFS = 7;

class PandaPinocchioModel
{
  public:
    PandaPinocchioModel(std::string urdf_filename, std::string ee_joint_name);
    JointPosition getJointAngleLowerLimits();
    JointPosition getJointAngleUpperLimits();
    JointVelocity getJointVelocityLimits();
    transform::Pose forwardKinematics(JointPosition joint_position, int64_t link_idx);
    transform::Pose forwardKinematics(JointPosition joint_position, const std::string& link_name);
    transform::Pose forwardKinematics(JointPosition joint_position);
    JocobianMatrix computeJacobian(JointPosition joint_position, int64_t link_idx);
    JocobianMatrix computeJacobian(JointPosition joint_position, const std::string& link_name);
    JocobianMatrix computeJacobian(JointPosition joint_position);
    JointTorque inverseDynamics(JointPosition joint_position, JointVelocity joint_velocity,
                                JointAcceleration joint_acceleration);
    Eigen::Matrix<double, 7, 7> mass(JointPosition joint_position);
    // JointPosition inverseKinematics(transform::Pose desired_pose, JointPosition initial_guess);

    // // Returns Coriolis + centrifugal torques for the given joint state.
    // JointTorque coriolis(JointPosition joint_position, JointVelocity joint_velocity);
    // // Returns gravity compensation torques for the given joint positions.
    // JointTorque gravity(JointPosition joint_position);

  private:
    void initialize();

    pinocchio::Model model_;
    pinocchio::Data model_data_;
    pinocchio::FrameIndex ee_idx_;

    std::string xml_buffer_;
    std::string ee_joint_name_;
    std::string ee_link_name;
    int64_t ee_link_idx_;
};
