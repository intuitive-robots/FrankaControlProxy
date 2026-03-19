#include "control_mode/cartesian_impedance.hpp"


void CartesianImpedanceController::startControl()
{
    config_.fromFile("config/controller/cartesian_impedance_controller.yaml");
    franka::RobotState curr_state = state_buffer_->read();
    Eigen::Affine3d T_EE_in_base_frame(Eigen::Matrix4d::Map(curr_state.O_T_EE.data()));
    desired_cartesian_pose_->write(transform::Pose(T_EE_in_base_frame));
    q_null_desired = Eigen::Matrix<double, 7, 1>{curr_state.q.data()};
    AbstractControlMode::startControl();
    zlc::info("[CartesianImpedance] Control started.");
}

franka::Torques CartesianImpedanceController::controlLoop(
    const franka::RobotState& robot_state,
    franka::Duration /*duration*/)
{
    // Get desired pose from trajectory interpolator
    Eigen::Vector3d desired_pos_EE;
    Eigen::Quaterniond desired_quat_EE;
    transform::Pose desired_pose = desired_cartesian_pose_->read();
    desired_pos_EE = desired_pose.translation();
    desired_quat_EE = desired_pose.quaternion();

    // Extract current joint state
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    JointPosition q_pin = Eigen::Map<const JointPosition>(robot_state.q.data());

    // Current end-effector pose
    Eigen::Affine3d T_EE_in_base(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d pos_current = T_EE_in_base.translation();
    Eigen::Quaterniond quat_current(T_EE_in_base.linear());

    // Compute Jacobian
    JocobianMatrix jacobian = pinocchio_model_->computeJacobian(q_pin);

    // Compute current EE twist (velocity): twist = J * dq
    Eigen::Matrix<double, 6, 1> twist_current = jacobian * dq;

    // Desired twist is zero
    Eigen::Matrix<double, 6, 1> twist_desired = Eigen::Matrix<double, 6, 1>::Zero();

    // ========== Compute pose error ==========
    Eigen::Vector3d pos_err = desired_pos_EE - pos_current;

    // Quaternion shortest path
    if (desired_quat_EE.coeffs().dot(quat_current.coeffs()) < 0.0)
    {
        quat_current.coeffs() << -quat_current.coeffs();
    }

    // Keep your original convention consistent
    Eigen::Quaterniond quat_err = quat_current.inverse() * desired_quat_EE;
    Eigen::Vector3d ori_err = quat_err.vec();
    ori_err = T_EE_in_base.linear() * ori_err;

    Eigen::Matrix<double, 6, 1> pose_err;
    pose_err << pos_err, ori_err;

    // ========== Compute twist error ==========
    Eigen::Matrix<double, 6, 1> twist_err = twist_desired - twist_current;

    // ========== Cartesian impedance wrench ==========
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Zero();
    Kp.block<3, 3>(0, 0) = config_.Kp_p;
    Kp.block<3, 3>(3, 3) = config_.Kp_r;

    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero();
    Kd.block<3, 3>(0, 0) = config_.Kd_p;
    Kd.block<3, 3>(3, 3) = config_.Kd_r;

    Eigen::Matrix<double, 6, 1> wrench_feedback = Kp * pose_err + Kd * twist_err;

    // ========== Main task torque ==========
    Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * wrench_feedback;

    // ========== Nullspace posture control ==========
    // Nullspace PD gains; better move these into config_
    double Kp_null = 10.0;
    double Kd_null = 2.0 * std::sqrt(Kp_null);

    // Joint posture control torque
    Eigen::Matrix<double, 7, 1> tau_null = Kp_null * (q_null_desired - q) - Kd_null * dq;

    // Jacobian pseudoinverse: J_pinv = J^T (J J^T)^-1
    // Use damped inverse for numerical robustness
    Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
    Eigen::Matrix<double, 6, 6> JJt_damped =
        JJt + config_.damping * Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 7, 6> J_pinv =
        jacobian.transpose() * JJt_damped.inverse();

    // Nullspace projector
    Eigen::Matrix<double, 7, 7> N =
        Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * jacobian;

    // Project posture torque into nullspace
    Eigen::Matrix<double, 7, 1> tau_null_projected = N * tau_null;

    // ========== Final torque ==========
    Eigen::Matrix<double, 7, 1> tau_d = tau_task + tau_null_projected;

    // Convert to std::array
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    // Rate limiting
    std::array<double, 7> tau_d_limited = tau_d_array;

    // Safety clamp
    for (size_t i = 0; i < 7; i++)
    {
        tau_d_limited[i] =
            std::clamp(tau_d_limited[i], -config_.max_torque, config_.max_torque);
    }

    return tau_d_limited;
}