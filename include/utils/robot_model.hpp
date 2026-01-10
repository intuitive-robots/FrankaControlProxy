#pragma once
#include <Eigen/Core>
#include <array>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <vector>

using JointPosition = Eigen::Vector<double, 7>;
using JointVelocity = Eigen::Vector<double, 7>;
using JointAcceleration = Eigen::Vector<double, 7>;
using JointTorque = Eigen::Vector<double, 7>;
using JocobianMatrix = Eigen::Matrix<double, 6, 7>;
using PoseRPY = Eigen::Vector<double, 6>;
using PoseQuat = Eigen::Vector<double, 7>;

class PandaPinocchioModel
{
  public:
    PandaPinocchioModel(std::string urdf_filename, std::string ee_joint_name);
    JointPosition getJointAngleLowerLimits();
    JointPosition getJointAngleUpperLimits();
    JointVelocity getJointVelocityLimits();
    JointPosition forwardKinematics(JointPosition joint_positions);
    JocobianMatrix computeJacobian(JointPosition joint_positions);
    JointPosition inverseDynamics(JointPosition joint_positions, JointVelocity joint_velocities,
                                  JointVelocity joint_accelerations);
    // Returns Coriolis + centrifugal torques for the given joint state.
    JointPosition coriolis(JointPosition joint_positions, JointVelocity joint_velocities);
    // Returns gravity compensation torques for the given joint positions.
    JointPosition gravity(JointPosition joint_positions);

  private:
    void initialize();

    pinocchio::Model model_;
    pinocchio::Data model_data_;
    pinocchio::FrameIndex ee_idx_;

    std::string xml_buffer_;
    std::string ee_joint_name_;
};
