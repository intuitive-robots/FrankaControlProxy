// #include "control_mode/cartesian_impedance_control.hpp"

// #include <Eigen/Geometry>
// #include <franka/exception.h>

// CartesianImpedanceControl::CartesianImpedanceControl()
//     : desired_pose_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
//       config_(CartesianImpedanceConfig())
// {
// }

// CartesianImpedanceControl::~CartesianImpedanceControl() = default;

// void CartesianImpedanceControl::initController()
// {
//     zlc::registerSubscriberHandler(config_.command_topic,
//                                    &CartesianImpedanceControl::writeCommand, this);
// }

// void CartesianImpedanceControl::writeCommand(const CartesianImpedanceCommand& cmd)
// {
//     desired_pose_.write(cmd.pose);
//     has_target_.store(true, std::memory_order_release);
// }

// franka::Torques CartesianImpedanceControl::controlLoop(const franka::RobotState& robot_state,
//                                                        franka::Duration /*duration*/)
// {
//     if (!robot_ || !model_ || !state_buffer_)
//     {
//         return franka::Torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//     }

//     state_buffer_->write(robot_state);

//     if (!has_target_.load(std::memory_order_acquire))
//     {
//         desired_pose_.write(robot_state);
//         has_target_.store(true, std::memory_order_release);
//     }

//     const std::array<double, 7> desired_pose = desired_pose_.read();
//     const std::array<double, 7> current_pose = model_->forwardKinematics(robot_state.q);

//     Eigen::Vector3d p_des(desired_pose[0], desired_pose[1], desired_pose[2]);
//     Eigen::Vector3d p_cur(current_pose[0], current_pose[1], current_pose[2]);
//     Eigen::Vector3d p_err = p_des - p_cur;

//     Eigen::Quaterniond q_des(desired_pose[6], desired_pose[3], desired_pose[4], desired_pose[5]);
//     Eigen::Quaterniond q_cur(current_pose[6], current_pose[3], current_pose[4], current_pose[5]);

//     Eigen::Quaterniond q_err = q_des * q_cur.conjugate();
//     if (q_err.w() < 0.0)
//     {
//         q_err.coeffs() *= -1.0;
//     }
//     Eigen::Vector3d o_err = 2.0 * q_err.vec();

//     Eigen::Matrix<double, 6, 1> error;
//     error << p_err, o_err;

//     Eigen::Matrix<double, 6, 7> J = model_->computeJacobian(robot_state.q);

//     Eigen::Matrix<double, 7, 1> dq;
//     for (int i = 0; i < 7; i++)
//     {
//         dq[i] = robot_state.dq[i];
//     }
//     Eigen::Matrix<double, 6, 1> ee_vel = J * dq;

//     Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
//     for (int i = 0; i < 6; i++)
//     {
//         wrench[i] = config_.k_gains[i] * error[i] - config_.d_gains[i] * ee_vel[i];
//     }

//     Eigen::Matrix<double, 7, 1> tau = J.transpose() * wrench;

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
//     for (int i = 0; i < 7; i++)
//     {
//         tau_cmd[i] = tau[i] + tau_ff[i];
//     }
//     return franka::Torques{tau_cmd};
// }
