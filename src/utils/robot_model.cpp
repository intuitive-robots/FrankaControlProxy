#include "utils/robot_model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iterator>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

PandaPinocchioModel::PandaPinocchioModel(std::string urdf_filename, std::string ee_joint_name)
{
    ee_joint_name_ = std::move(ee_joint_name);

    std::ifstream stream(urdf_filename);
    xml_buffer_ =
        std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

    initialize();
}

void PandaPinocchioModel::initialize()
{
    pinocchio::urdf::buildModelFromXML(xml_buffer_, model_);
    model_data_ = pinocchio::Data(model_);
    ee_idx_ = model_.getFrameId(ee_joint_name_);
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

JointPosition PandaPinocchioModel::forwardKinematics(JointPosition joint_positions)
{
    JointPosition q = joint_positions;

    pinocchio::forwardKinematics(model_, model_data_, q);
    pinocchio::updateFramePlacement(model_, model_data_, ee_idx_);

    const auto& pos = model_data_.oMf[ee_idx_].translation();
    Eigen::Quaterniond quat(model_data_.oMf[ee_idx_].rotation());

    JointPosition result{};
    result << pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w();
    return result;
}

JocobianMatrix PandaPinocchioModel::computeJacobian(JointPosition joint_positions)
{
    JointPosition q = joint_positions;

    JocobianMatrix J = JocobianMatrix::Zero();
    pinocchio::computeFrameJacobian(model_, model_data_, q, ee_idx_, pinocchio::LOCAL_WORLD_ALIGNED,
                                    J);
    return J;
}

JointPosition PandaPinocchioModel::inverseDynamics(JointPosition joint_positions,
                                                   JointVelocity joint_velocities,
                                                   JointVelocity joint_accelerations)
{
    JointPosition q = joint_positions;
    JointVelocity v = joint_velocities;
    JointVelocity a = joint_accelerations;

    Eigen::Matrix<double, Eigen::Dynamic, 1> tau = pinocchio::rnea(model_, model_data_, q, v, a);

    JointPosition result{};
    result = tau;
    return result;
}

JointPosition PandaPinocchioModel::coriolis(JointPosition joint_positions,
                                            JointVelocity joint_velocities)
{
    JointPosition q = joint_positions;
    JointVelocity v = joint_velocities;

    const auto nle = pinocchio::nonLinearEffects(model_, model_data_, q, v);
    const auto g = pinocchio::computeGeneralizedGravity(model_, model_data_, q);

    JointPosition result{};
    result = nle - g;
    return result;
}

JointPosition PandaPinocchioModel::gravity(JointPosition joint_positions)
{
    JointPosition q = joint_positions;

    const auto g = pinocchio::computeGeneralizedGravity(model_, model_data_, q);

    JointPosition result{};
    result = g;
    return result;
}
