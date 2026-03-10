#include "control_mode/cartesian_impedance.hpp"

#include <franka/rate_limiting.h>

#include <Eigen/Dense>

#include "utils/Pose.h"

// void CartesianImpedanceController::initController(
//     FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
//     AtomicDoubleBuffer<franka::RobotState>& state_buffer)
// {
//     AbstractControlMode::initController(robot, pinocchio_model, state_buffer);
//     robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
//     robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

//     robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
//     config_.fromFile("config/controller/cartesian_impedance_controller.cfg");
//     franka::RobotState curr_state = state_buffer_->read();
//     Eigen::Affine3d T_EE_in_base_frame(Eigen::Matrix4d::Map(curr_state.O_T_EE.data()));
//     Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
//     Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());
//     pos_EE_in_base_frame =
//         pos_EE_in_base_frame + Eigen::Vector3d(0.0, 0.2, 0.1); // Add offset to avoid singularity
//     traj_interpolator.reset(0., pos_EE_in_base_frame, quat_EE_in_base_frame, pos_EE_in_base_frame,
//                             quat_EE_in_base_frame, 20, 500, 1.0);
//     zlc::info("[CartesianImpedance] Initialized.");
// }

void CartesianImpedanceController::startControl()
{
    config_.fromFile("config/controller/cartesian_impedance_controller.cfg");
    franka::RobotState curr_state = state_buffer_->read();
    Eigen::Affine3d T_EE_in_base_frame(Eigen::Matrix4d::Map(curr_state.O_T_EE.data()));
    desired_cartesian_pose_->write(transform::Pose(T_EE_in_base_frame));
    AbstractControlMode::startControl();
    zlc::info("[CartesianImpedance] Control started.");
}

franka::Torques CartesianImpedanceController::controlLoop(const franka::RobotState& robot_state,
                                                          franka::Duration period)
{
    // Get desired pose from trajectory interpolator
    Eigen::Vector3d desired_pos_EE;
    Eigen::Quaterniond desired_quat_EE;
    transform::Pose desired_pose = desired_cartesian_pose_->read();
    desired_pos_EE = desired_pose.translation();
    desired_quat_EE = desired_pose.quaternion();

    // Extract current joint state
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    JointPosition q_pin = Eigen::Map<const JointPosition>(robot_state.q.data());

    // Current end-effector pose
    Eigen::Affine3d T_EE_in_base(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d pos_current = T_EE_in_base.translation();
    Eigen::Quaterniond quat_current(T_EE_in_base.linear());

    // Compute Jacobian
    JocobianMatrix jacobian = pinocchio_model_->computeJacobian(q_pin);

    // Compute current EE twist (velocity): twist = J * dq
    Eigen::Matrix<double, 6, 1> twist_current = jacobian * dq;

    // Desired twist is zero (we want to reach and stop at the desired pose)
    Eigen::Matrix<double, 6, 1> twist_desired = Eigen::Matrix<double, 6, 1>::Zero();

    // ========== Compute pose error ==========
    // Position error
    Eigen::Vector3d pos_err = desired_pos_EE - pos_current;

    // Orientation error (from libfranka cartesian impedance example)
    // Ensure quaternion shortest path
    if (desired_quat_EE.coeffs().dot(quat_current.coeffs()) < 0.0)
    {
        quat_current.coeffs() << -quat_current.coeffs();
    }
    Eigen::Quaterniond quat_err = quat_current.inverse() * desired_quat_EE;
    Eigen::Vector3d ori_err = quat_err.vec();  // Extract imaginary part
    ori_err = T_EE_in_base.linear() * ori_err; // Transform to base frame

    // Combine into 6D pose error [pos_err; ori_err]
    Eigen::Matrix<double, 6, 1> pose_err;
    pose_err << pos_err, ori_err;

    // ========== Compute twist error ==========
    Eigen::Matrix<double, 6, 1> twist_err = twist_desired - twist_current;

    // ========== Compute wrench feedback ==========
    // wrench = Kp * pose_err + Kd * twist_err
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Zero();
    Kp.block<3, 3>(0, 0) = config_.Kp_p;
    Kp.block<3, 3>(3, 3) = config_.Kp_r;

    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero();
    Kd.block<3, 3>(0, 0) = config_.Kd_p;
    Kd.block<3, 3>(3, 3) = config_.Kd_r;

    Eigen::Matrix<double, 6, 1> wrench_feedback = Kp * pose_err + Kd * twist_err;

    // ========== Project wrench to joint torques ==========
    // torque_feedback = J^T * wrench
    Eigen::Matrix<double, 7, 1> tau_feedback = jacobian.transpose() * wrench_feedback;

    // ========== Compute feedforward (Coriolis compensation) ==========
    Eigen::Matrix<double, 7, 1> tau_coriolis;
    if (!config_.ignore_gravity)
    {
        // Full inverse dynamics (gravity + coriolis)
        tau_coriolis =
            pinocchio_model_->inverseDynamics(q_pin, dq, Eigen::Matrix<double, 7, 1>::Zero());
    }
    else
    {
        // Only coriolis (robot already compensates gravity internally)
        tau_coriolis =
            pinocchio_model_->inverseDynamics(q_pin, dq, Eigen::Matrix<double, 7, 1>::Zero());
    }

    // ========== Total torque ==========
    Eigen::Matrix<double, 7, 1> tau_d = tau_feedback;

    // Convert to std::array
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    // Rate limiting
    std::array<double, 7> tau_d_limited = tau_d_array;

    // Safety clamp
    double min_torque = -50.0;
    double max_torque = 50.0;
    for (size_t i = 0; i < 7; i++)
    {
        tau_d_limited[i] = std::clamp(tau_d_limited[i], min_torque, max_torque);
    }

    return tau_d_limited;
}
