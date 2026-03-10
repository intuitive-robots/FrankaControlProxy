#include "control_mode/osc_control.hpp"
#include <franka/rate_limiting.h>
#include <Eigen/Dense>
#include "utils/Pose.h"

void PInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv,
              double epsilon) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_vals =
      svd.singularValues();

  Eigen::MatrixXd S_inv = M;
  S_inv.setZero();
  for (int i = 0; i < singular_vals.size(); i++) {
    if (singular_vals(i) < epsilon) {
      S_inv(i, i) = 0.;
    } else {
      S_inv(i, i) = 1. / singular_vals(i);
    }
  }
  M_inv = Eigen::MatrixXd(svd.matrixV() * S_inv * svd.matrixU().transpose());
}

void TorqueSafetyGuardFn(std::array<double, 7>& tau_d_array, double min_torque,
                         double max_torque) {
  for (size_t i = 0; i < tau_d_array.size(); i++) {
    if (tau_d_array[i] < min_torque) {
      tau_d_array[i] = min_torque;
    } else if (tau_d_array[i] > max_torque) {
      tau_d_array[i] = max_torque;
    }
  }
}

void OSCController::initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                                        AtomicDoubleBuffer<franka::RobotState>& state_buffer)
{
    AbstractControlMode::initController(robot, pinocchio_model, state_buffer);
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

    // high collision threshold values for high impedance
    robot_->setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    franka::RobotState curr_state = state_buffer_->read();
    // use current pose as initial pose for interpolation
    Eigen::Affine3d T_EE_in_base_frame(
        Eigen::Matrix4d::Map(curr_state.O_T_EE.data()));
    Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
    std::cout << "Initial EE position: " << pos_EE_in_base_frame.transpose() << std::endl;
    Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());
    traj_interpolator.reset(0., pos_EE_in_base_frame, quat_EE_in_base_frame, pos_EE_in_base_frame, quat_EE_in_base_frame, 20, 500, 1.0);
    zlc::info("[OSC] Initialized (spring + damping, no gravity compensation).");
}

void OSCController::writeCommand(const CartesianPoseCommand& cmd)
{
    franka::RobotState curr_state = state_buffer_->read();
    Eigen::Vector3d desired_EE_pos(cmd.pos.data());
    Eigen::Quaterniond desired_EE_quat(cmd.quat.data());
    rcs::common::Pose desired_pose_EE_in_base_frame(desired_EE_quat, desired_EE_pos);
    rcs::common::Pose curr_pose(curr_state.O_T_EE);
    traj_interpolator.reset(
      controller_time, curr_pose.translation(), curr_pose.quaternion(),
      desired_pose_EE_in_base_frame.translation(),
      desired_pose_EE_in_base_frame.quaternion(), 20, 500,
      1.0);
}

franka::Torques OSCController::controlLoop(const franka::RobotState& robot_state,
                                                franka::Duration period)
{
     std::chrono::high_resolution_clock::time_point t1 =
          std::chrono::high_resolution_clock::now();

      Eigen::Vector3d desired_pos_EE_in_base_frame;
      Eigen::Quaterniond desired_quat_EE_in_base_frame;

    //   common::Pose pose(robot_state.O_T_EE);
      // form deoxys/config/charmander.yml
      int policy_rate = 20;
      int traj_rate = 500;

      controller_time += period.toSec();
      this->traj_interpolator.next_step(this->controller_time,
                                        desired_pos_EE_in_base_frame,
                                        desired_quat_EE_in_base_frame);
      Eigen::Matrix<double, 7, 1> tau_d;

      // Extract joint position for pinocchio model
      JointPosition q_pin = Eigen::Map<const JointPosition>(robot_state.q.data());

      // Mass matrix from Pinocchio
      Eigen::Matrix<double, 7, 7> M = pinocchio_model_->mass(q_pin);
      M = M + Eigen::Matrix<double, 7, 7>(config_.residual_mass_vec.asDiagonal());

      // Jacobian from Pinocchio
      JocobianMatrix jacobian = pinocchio_model_->computeJacobian(q_pin);

      Eigen::MatrixXd jacobian_pos(3, 7);
      Eigen::MatrixXd jacobian_ori(3, 7);
      jacobian_pos << jacobian.block(0, 0, 3, 7);
      jacobian_ori << jacobian.block(3, 0, 3, 7);

      // End effector pose in base frame
      Eigen::Affine3d T_EE_in_base_frame(
          Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
      Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());

      // Nullspace goal
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      // Joint velocity
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      if (desired_quat_EE_in_base_frame.coeffs().dot(
              quat_EE_in_base_frame.coeffs()) < 0.0) {
        quat_EE_in_base_frame.coeffs() << -quat_EE_in_base_frame.coeffs();
      }

      Eigen::Vector3d pos_error;

      pos_error << desired_pos_EE_in_base_frame - pos_EE_in_base_frame;
      std::cout << "Position error: " << pos_error.transpose() << std::endl;
      Eigen::Quaterniond quat_error(desired_quat_EE_in_base_frame.inverse() *
                                    quat_EE_in_base_frame);
      Eigen::Vector3d ori_error;
      ori_error << quat_error.x(), quat_error.y(), quat_error.z();
      ori_error << -T_EE_in_base_frame.linear() * ori_error;

      // Compute matrices
      Eigen::Matrix<double, 7, 7> M_inv(M.inverse());
      Eigen::MatrixXd Lambda_inv(6, 6);
      Lambda_inv << jacobian * M_inv * jacobian.transpose();
      Eigen::MatrixXd Lambda(6, 6);
      PInverse(Lambda_inv, Lambda);

      Eigen::Matrix<double, 7, 6> J_inv;
      J_inv << M_inv * jacobian.transpose() * Lambda;
      Eigen::Matrix<double, 7, 7> Nullspace;
      Nullspace << Eigen::MatrixXd::Identity(7, 7) -
                       jacobian.transpose() * J_inv.transpose();

      // Decoupled mass matrices
      Eigen::MatrixXd Lambda_pos_inv(3, 3);
      Lambda_pos_inv << jacobian_pos * M_inv * jacobian_pos.transpose();
      Eigen::MatrixXd Lambda_ori_inv(3, 3);
      Lambda_ori_inv << jacobian_ori * M_inv * jacobian_ori.transpose();

      Eigen::MatrixXd Lambda_pos(3, 3);
      Eigen::MatrixXd Lambda_ori(3, 3);
      PInverse(Lambda_pos_inv, Lambda_pos);
      PInverse(Lambda_ori_inv, Lambda_ori);

      pos_error = pos_error.unaryExpr(
          [](double x) { return (abs(x) < 1e-4) ? 0. : x; });
      ori_error = ori_error.unaryExpr(
          [](double x) { return (abs(x) < 5e-3) ? 0. : x; });

      tau_d << jacobian_pos.transpose() *
                       (Lambda_pos *
                        (config_.Kp_p * pos_error - config_.Kd_p * (jacobian_pos * dq))) +
                   jacobian_ori.transpose() *
                       (Lambda_ori *
                        (config_.Kp_r * ori_error - config_.Kd_r * (jacobian_ori * dq)));

      // nullspace control
      tau_d << tau_d + Nullspace * (config_.static_q_task - q);

      // Add joint avoidance potential
      Eigen::Matrix<double, 7, 1> avoidance_force;
      avoidance_force.setZero();
      Eigen::Matrix<double, 7, 1> dist2joint_max;
      Eigen::Matrix<double, 7, 1> dist2joint_min;

      dist2joint_max = config_.joint_max.matrix() - q;
      dist2joint_min = q - config_.joint_min.matrix();

      for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.25 && dist2joint_max[i] > 0.1)
          avoidance_force[i] += -config_.avoidance_weights[i] * dist2joint_max[i];
        if (dist2joint_min[i] < 0.25 && dist2joint_min[i] > 0.1)
          avoidance_force[i] += config_.avoidance_weights[i] * dist2joint_min[i];
      }
      tau_d << tau_d + Nullspace * avoidance_force;
      for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.) tau_d[i] = 0.;
        if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.) tau_d[i] = 0.;
      }

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // end of controller
      std::chrono::high_resolution_clock::time_point t2 =
          std::chrono::high_resolution_clock::now();
      auto time =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

      std::array<double, 7> tau_d_rate_limited = franka::limitRate(
          franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);

      double min_torque = -5;
      double max_torque = 5;
      TorqueSafetyGuardFn(tau_d_rate_limited, min_torque, max_torque);

      return tau_d_rate_limited;
}

void OSCController::startControl() {
    franka::RobotState curr_state = state_buffer_->read();
    Eigen::Affine3d T_EE_in_base_frame(
        Eigen::Matrix4d::Map(curr_state.O_T_EE.data()));
    Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
    Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());
    traj_interpolator.reset(0., pos_EE_in_base_frame, quat_EE_in_base_frame, pos_EE_in_base_frame, quat_EE_in_base_frame, 20, 500, 1.0);
    AbstractControlMode::startControl();
    std::cout << "[OSC] Control started with initial pose as setpoint." << pos_EE_in_base_frame.transpose() << std::endl;
}