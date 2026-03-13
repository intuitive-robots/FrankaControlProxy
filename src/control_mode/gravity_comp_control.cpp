// #include "control_mode/gravity_comp_control.hpp"

// #include <Eigen/Dense>

// void GravityCompControl::reset()
// {
//     hold_initialized_ = false;
// }

// void GravityCompControl::initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
//                                         AtomicDoubleBuffer<franka::RobotState>& state_buffer)
// {
//     AbstractControlMode::initController(robot, pinocchio_model, state_buffer);
//     controller_name = "GravityComp";
//     reset();
//     zlc::info("[GravityComp] Initialized (spring + damping, no gravity compensation).");
// }

// franka::Torques GravityCompControl::controlLoop(const franka::RobotState& robot_state,
//                                                 franka::Duration /*duration*/)
// {
//     // Keep publishing state for others (e.g. follower) even when leader is hand-guided.
//     state_buffer_->write(robot_state);

//     // Map robot state to Eigen vectors
//     JointPosition q = Eigen::Map<const JointPosition>(robot_state.q.data());
//     JointVelocity dq = Eigen::Map<const JointVelocity>(robot_state.dq.data());

//     if (!hold_initialized_)
//     {
//         hold_q_ = q;
//         hold_initialized_ = true;
//     }

//     // Simple spring + damping (no gravity compensation).
//     // Tune these two values to change hand-guiding feel.
//     const std::array<double, 7> k_hold = {0.08, 0.08, 0.08, 0.04, 0.04, 0.08, 0.08};
//     const std::array<double, 7> damping = {0.5, 0.5, 0.5, 0.4, 0.3, 0.5, 0.5};

//     std::array<double, 7> tau_cmd{};
//     for (int i = 0; i < 7; i++)
//     {
//         const double tau_hold = k_hold[i] * (hold_q_[i] - q[i]);
//         const double tau = tau_hold - damping[i] * dq[i];
//         tau_cmd[i] = tau;
//     }

//     franka::Torques out{tau_cmd};
//     if (!is_running_)
//     {
//         out = franka::Torques{};
//         out.motion_finished = true;
//     }
//     return out;
// }
