#pragma once

#include <array>
#include <atomic>
#include <string>
#include <utility>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "control_mode/joint_target.hpp"
#include "protocol/control_command.hpp"
#include "utils/Pose.h"

struct HybridJointImpedanceConfig : public ControllerConfig
{
    Eigen::Matrix<double, 7, 7> kq;
    Eigen::Matrix<double, 7, 7> kqd;
    Eigen::Matrix<double, 6, 6> kx;
    Eigen::Matrix<double, 6, 6> kxd;
    bool ignore_gravity{true};
    JointFeedforwardConfig feedforward;
    HybridJointImpedanceConfig() = default;

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
        feedforward.read(reader);
        max_ee_force = reader.getValue<double>("max_ee_force", 30.0);
        max_ee_torque = reader.getValue<double>("max_ee_torque", 3.0);
        ee_wrench_timeout = reader.getValue<double>("ee_wrench_timeout", 0.1);
    }

    // Feed-forward wrench command (EEWrenchCommand): norm limits and the age after which it is dropped
    double max_ee_force{30.0};
    double max_ee_torque{3.0};
    double ee_wrench_timeout{0.1};
};

// Feed-forward wrench at the flange (base frame) with its receive time, e.g. to push during insertion
struct EEWrenchTarget
{
    transform::Vector6d wrench{transform::Vector6d::Zero()};
    std::chrono::steady_clock::time_point stamp{};
};

class HybridJointImpedanceControl : public AbstractControlMode
{
  public:
    // config_path selects the gain set, so one class serves several registered modes
    // (e.g. stiff and compliant teleoperation)
    HybridJointImpedanceControl(
        AtomicDoubleBuffer<JointTarget>& desired_joint_command_,
        AtomicDoubleBuffer<EEWrenchTarget>& desired_ee_wrench_,
        std::string config_path = "config/controller/hybrid_joint_impedance_controller.yaml")
        : desired_joint_command_(&desired_joint_command_)
        , desired_ee_wrench_(&desired_ee_wrench_)
        , config_path_(std::move(config_path))
    {
        controller_name = "HybridJointImpedanceControl";
    };
    ~HybridJointImpedanceControl() override;

  private:
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
    void startControl() override;
    HybridJointImpedanceConfig config_;
    std::string config_path_;
    AtomicDoubleBuffer<JointTarget>* desired_joint_command_;
    AtomicDoubleBuffer<EEWrenchTarget>* desired_ee_wrench_;
    JointReferenceGenerator reference_generator_;
    // J^T * wrench of a recent wrench command, clamped to the configured limits (else zero)
    JointTorque eeWrenchTorque(const JocobianMatrix& J) const;
};
