#include "utils/robot_model.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <fstream>
#include <iostream>

PandaPinocchioModel::PandaPinocchioModel(std::string urdf_filename, std::string ee_joint_name)
{
    ee_joint_name_ = std::move(ee_joint_name);

    std::ifstream stream(urdf_filename);
    if (!stream.is_open()) {
        throw std::runtime_error("Cannot open URDF file: " + urdf_filename);
    }
    
    xml_buffer_ =
        std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    pinocchio::urdf::buildModelFromXML(xml_buffer_, model_);
    model_data_ = pinocchio::Data(model_);
    
    // Check if the frame exists
    if (!model_.existFrame(ee_joint_name_)) {
        std::cerr << "Frame '" << ee_joint_name_ << "' not found in model!" << std::endl;
        std::cerr << "Available frames:" << std::endl;
        for (size_t i = 0; i < model_.nframes; ++i) {
            std::cerr << "  " << i << ": " << model_.frames[i].name << std::endl;
        }
        throw std::runtime_error("Frame '" + ee_joint_name_ + "' not found in URDF model");
    }
    
    ee_link_idx_ = model_.getFrameId(ee_joint_name_);
}

JointPosition PandaPinocchioModel::getJointAngleLowerLimits()
{
    JointPosition result{};
    for (int i = 0; i < 7; i++)
    {
        result[i] = model_.lowerPositionLimit[i];
    }
    return result;
}

JointPosition PandaPinocchioModel::getJointAngleUpperLimits()
{
    JointPosition result{};
    for (int i = 0; i < 7; i++)
    {
        result[i] = model_.upperPositionLimit[i];
    }
    return result;
}

JointVelocity PandaPinocchioModel::getJointVelocityLimits()
{
    JointVelocity result{};
    for (int i = 0; i < 7; i++)
    {
        result[i] = model_.velocityLimit[i];
    }
    return result;
}

PoseQuat PandaPinocchioModel::forwardKinematics(JointPosition joint_positions, int64_t link_idx)
{
    pinocchio::FrameIndex frame_idx = static_cast<pinocchio::FrameIndex>(link_idx);

    pinocchio::forwardKinematics(model_, model_data_, joint_positions);
    pinocchio::updateFramePlacement(model_, model_data_, frame_idx);

    auto pos_data = model_data_.oMf[frame_idx].translation().transpose();
    auto quat_data = Eigen::Quaterniond(model_data_.oMf[frame_idx].rotation());

    PoseQuat result;
    for (int i = 0; i < 3; i++)
    {
        result[i] = pos_data[i];
    }
    result[3] = quat_data.x();
    result[4] = quat_data.y();
    result[5] = quat_data.z();
    result[6] = quat_data.w();
    return result;
}

PoseQuat PandaPinocchioModel::forwardKinematics(JointPosition joint_positions,
                                                const std::string& link_name)
{
    pinocchio::FrameIndex frame_idx = model_.getFrameId(link_name);
    return forwardKinematics(joint_positions, frame_idx);
}

PoseQuat PandaPinocchioModel::forwardKinematics(JointPosition joint_positions)
{
    return forwardKinematics(joint_positions, ee_link_idx_);
}

JocobianMatrix PandaPinocchioModel::computeJacobian(JointPosition joint_positions, int64_t link_idx)
{
    JocobianMatrix J = JocobianMatrix::Zero();
    pinocchio::FrameIndex frame_idx = static_cast<pinocchio::FrameIndex>(link_idx);
    pinocchio::computeFrameJacobian(model_, model_data_, joint_positions, frame_idx,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}

JocobianMatrix PandaPinocchioModel::computeJacobian(JointPosition joint_positions,
                                                    const std::string& link_name)
{
    pinocchio::FrameIndex frame_idx = model_.getFrameId(link_name);
    return computeJacobian(joint_positions, frame_idx);
}

JocobianMatrix PandaPinocchioModel::computeJacobian(JointPosition joint_positions)
{
    return computeJacobian(joint_positions, ee_link_idx_);
}

JointPosition PandaPinocchioModel::inverseDynamics(JointPosition joint_position,
                                                   JointVelocity joint_velocity,
                                                   JointAcceleration joint_acceleration)
{
    // If model has more than 7 DOF (e.g., with gripper), use only first 7
    if (model_.nv > 7)
    {
        // Pad joint position, velocity, and acceleration to match model DOF
        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nv);
        Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
        Eigen::VectorXd ddq_full = Eigen::VectorXd::Zero(model_.nv);
        
        q_full.head(7) = joint_position;
        dq_full.head(7) = joint_velocity;
        ddq_full.head(7) = joint_acceleration;
        
        Eigen::VectorXd tau_full = pinocchio::rnea(model_, model_data_, q_full, dq_full, ddq_full);
        
        // Return only first 7 elements
        return tau_full.head(7);
    }
    else
    {
        return pinocchio::rnea(model_, model_data_, joint_position, joint_velocity, joint_acceleration);
    }
}

// JointPosition PandaPinocchioModel::coriolis(JointPosition joint_position,
//                                             JointVelocity joint_velocity)
// {
//     return pinocchio::rnea(model_, model_data_, joint_positions, joint_velocitie, a);
// }

// JointPosition PandaPinocchioModel::gravity(JointPosition joint_positions)
// {
//     JointPosition q = joint_positions;

//     const auto g = pinocchio::computeGeneralizedGravity(model_, model_data_, q);

//     JointPosition result{};
//     result = g;
//     return result;
// }
