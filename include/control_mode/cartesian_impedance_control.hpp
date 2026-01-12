// #pragma once

// #include <array>
// #include <atomic>
// #include <msgpack.hpp>

// #include "control_mode/abstract_control_mode.hpp"

// struct CartesianImpedanceCommand
// {
//     std::array<double, 7> pose;
//     MSGPACK_DEFINE_MAP(pose)
// };

// struct CartesianImpedanceConfig
// {
//     std::string command_topic{"FRANKA_CARTESIAN_IMPEDANCE_CMD"};
//     std::array<double, 6> k_gains{{150.0, 150.0, 150.0, 20.0, 20.0, 20.0}};
//     std::array<double, 6> d_gains{{20.0, 20.0, 20.0, 2.0, 2.0, 2.0}};
//     bool ignore_gravity{true};
// };

// class CartesianImpedanceControl : public AbstractControlMode
// {
//   public:
//     CartesianImpedanceControl();
//     ~CartesianImpedanceControl() override;

//   private:
//     franka::Torques controlLoop(const franka::RobotState& robot_state,
//                                 franka::Duration duration) override;
//     void writeCommand(const CartesianImpedanceCommand& cmd);

//     AtomicDoubleBuffer<std::array<double, 7>> desired_pose_;
//     std::atomic<bool> has_target_{false};
//     CartesianImpedanceConfig config_;
// };
