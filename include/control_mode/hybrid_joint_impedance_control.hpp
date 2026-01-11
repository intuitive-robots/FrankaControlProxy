#pragma once

#include <array>
#include <atomic>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"

struct HybridJointImpedanceCommand
{
    std::array<double, 7> pos;
    MSGPACK_DEFINE_MAP(pos);
};

struct HybridJointImpedanceConfig : public ControllerConfig
{
    std::string command_topic{"FRANKA_HYBRID_JOINT_IMPEDANCE_CMD"};
    Eigen::Matrix<double, 7, 7> kq;
    Eigen::Matrix<double, 7, 7> kqd;
    Eigen::Matrix<double, 6, 6> kx;
    Eigen::Matrix<double, 6, 6> kxd;
    bool ignore_gravity{true};
    HybridJointImpedanceConfig() = default;

    void fromFile(const std::string& controller_config_path) override
    {
        ConfigFileReader reader(controller_config_path);
        readBaseConfig(reader);
        command_topic = reader.getValue<std::string>("command_topic");
        const std::array<double, 7> kq_gains = reader.getArray<double, 7>("kq_gains");
        kq = JointPosition::Map(kq_gains.data()).asDiagonal();
        const std::array<double, 7> kqd_gains = reader.getArray<double, 7>("kqd_gains");
        kqd = JointVelocity::Map(kqd_gains.data()).asDiagonal();
        const std::array<double, 6> kx_gains = reader.getArray<double, 6>("kx_gains");
        kx = PoseRPY::Map(kx_gains.data()).asDiagonal();
        const std::array<double, 6> kxd_gains = reader.getArray<double, 6>("kxd_gains");
        kxd = PoseRPY::Map(kxd_gains.data()).asDiagonal();
        ignore_gravity = reader.getValue<bool>("ignore_gravity");
    }
};

class HybridJointImpedanceControl : public AbstractControlMode
{
  public:
    // HybridJointImpedanceControl() : AbstractControlMode(robot, model, state_buffer) {
    // };
    ~HybridJointImpedanceControl() override;

  private:
    void initController(FrankaPanda& robot, PandaPinocchioModel& model,
                        AtomicDoubleBuffer<franka::RobotState>& state_buffer) override;
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
    void writeCommand(const HybridJointImpedanceCommand& cmd);

    AtomicDoubleBuffer<JointPosition> desired_positions_{JointPosition::Zero()};
    HybridJointImpedanceConfig config_;
};
