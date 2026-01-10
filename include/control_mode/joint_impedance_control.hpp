// #pragma once

// #include <array>
// #include <atomic>
// #include <msgpack.hpp>

// #include "control_mode/abstract_control_mode.hpp"

// struct JointImpedanceCommand
// {
//     std::array<double, 7> joint_pos;
//     MSGPACK_DEFINE_MAP(joint_pos);
// };

// struct JointImpedanceConfig
// {
//     std::string command_topic{"FRANKA_JOINT_IMPEDANCE_CMD"};
//     std::array<double, 7> k_gains{{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
//     std::array<double, 7> d_gains{{20.0, 20.0, 20.0, 20.0, 12.0, 8.0, 5.0}};
//     bool ignore_gravity{true};
// };

// class JointImpedanceControl : public AbstractControlMode
// {
//   public:
//     JointImpedanceControl();
//     ~JointImpedanceControl() override;

//   private:
//     void initController() override;
//     franka::Torques controlLoop(const franka::RobotState& robot_state,
//                                 franka::Duration duration) override;
//     void writeCommand(const JointImpedanceCommand& cmd);

//     AtomicDoubleBuffer<std::array<double, 7>> desired_positions_;
//     std::atomic<bool> has_target_{false};
//     JointImpedanceConfig config_;
// };
