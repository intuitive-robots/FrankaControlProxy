#pragma once

#include <array>
#include <atomic>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "protocol/control_command.hpp"
#include "utils/Pose.h"

struct HybridJointExtTorqueImpedanceConfig : public ControllerConfig
{
    Eigen::Matrix<double, 7, 7> kq;
    Eigen::Matrix<double, 7, 7> kqd;
    Eigen::Matrix<double, 6, 6> kx;
    Eigen::Matrix<double, 6, 6> kxd;
    bool ignore_gravity{true};
    HybridJointExtTorqueImpedanceConfig() = default;

    void fromFile(const std::string& controller_config_path) override
    {
        ConfigFileReader reader(controller_config_path);
        readBaseConfig(reader);
        const std::array<double, 7> kq_gains = reader.getArray<double, 7>("kq_gains");
        kq = JointPosition::Map(kq_gains.data()).asDiagonal();
        const std::array<double, 7> kqd_gains = reader.getArray<double, 7>("kqd_gains");
        kqd = JointVelocity::Map(kqd_gains.data()).asDiagonal();
        const std::array<double, 6> kx_gains = reader.getArray<double, 6>("kx_gains");
        kx = transform::Vector6d::Map(kx_gains.data()).asDiagonal();
        const std::array<double, 6> kxd_gains = reader.getArray<double, 6>("kxd_gains");
        kxd = transform::Vector6d::Map(kxd_gains.data()).asDiagonal();
        ignore_gravity = reader.getValue<bool>("ignore_gravity");
    }
};

class HybridJointExtTorqueImpedanceControl : public AbstractControlMode
{
  public:
    HybridJointExtTorqueImpedanceControl(AtomicDoubleBuffer<JointPosition>& desired_joint_command_,
                                         AtomicDoubleBuffer<JointTorque>& desired_ext_tau)
        : desired_joint_command_(&desired_joint_command_)
        , desired_ext_tau_(&desired_ext_tau)
    {
        controller_name = "HybridJointExtTorqueImpedanceControl";
    };
    ~HybridJointExtTorqueImpedanceControl() override;

  private:
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
    void startControl() override;
    HybridJointExtTorqueImpedanceConfig config_;
    AtomicDoubleBuffer<JointPosition>* desired_joint_command_;
    AtomicDoubleBuffer<JointTorque>* desired_ext_tau_;
};