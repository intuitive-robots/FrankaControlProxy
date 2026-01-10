// #include "control_mode/joint_impedance_control.hpp"

// #include <franka/exception.h>

// JointImpedanceControl::JointImpedanceControl()
//     : desired_positions_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), config_(JointImpedanceConfig())
// {
// }

// JointImpedanceControl::~JointImpedanceControl() = default;

// void JointImpedanceControl::initController()
// {
//     zlc::registerSubscriberHandler(config_.command_topic, &JointImpedanceControl::writeCommand,
//                                    this);
// }

// void JointImpedanceControl::writeCommand(const JointImpedanceCommand& cmd)
// {
//     desired_positions_.write(cmd.joint_pos);
//     has_target_.store(true, std::memory_order_release);
// }

// franka::Torques JointImpedanceControl::controlLoop(const franka::RobotState& robot_state,
//                                                    franka::Duration /*duration*/)
// {
//     if (!robot_ || !model_ || !state_buffer_)
//     {
//         return franka::Torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//     }

//     state_buffer_->write(robot_state);

//     if (!has_target_.load(std::memory_order_acquire))
//     {
//         desired_positions_.write(robot_state.q);
//         has_target_.store(true, std::memory_order_release);
//     }

//     const std::array<double, 7> q_des = desired_positions_.read();
//     std::array<double, 7> tau_fb{};
//     for (size_t i = 0; i < 7; i++)
//     {
//         tau_fb[i] = config_.k_gains[i] * (q_des[i] - robot_state.q[i]) -
//                     config_.d_gains[i] * robot_state.dq[i];
//     }

//     std::array<double, 7> tau_ff{};
//     if (config_.ignore_gravity)
//     {
//         tau_ff = model_->coriolis(robot_state.q, robot_state.dq);
//     }
//     else
//     {
//         tau_ff = model_->inverseDynamics(robot_state.q, robot_state.dq,
//                                          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//     }

//     std::array<double, 7> tau_cmd{};
//     for (size_t i = 0; i < 7; i++)
//     {
//         tau_cmd[i] = tau_fb[i] + tau_ff[i];
//     }
//     return franka::Torques{tau_cmd};
// }
