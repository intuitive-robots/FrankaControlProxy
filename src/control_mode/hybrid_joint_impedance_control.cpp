#include "control_mode/hybrid_joint_impedance_control.hpp"

#include <Eigen/Geometry>

HybridJointImpedanceControl::~HybridJointImpedanceControl() = default;


void HybridJointImpedanceControl::startControl()
{
    config_.fromFile(config_path_);
    std::array<double, 7> current_pos = state_buffer_->read().q;
    JointTarget target;
    target.pos = JointPosition::Map(current_pos.data());
    desired_joint_command_->write(target);
    reference_generator_.reset(target.pos, config_.feedforward);
    // a wrench left over from a previous mode must not push when this one starts
    desired_ee_wrench_->write(EEWrenchTarget{});
    AbstractControlMode::startControl();
}

JointTorque HybridJointImpedanceControl::eeWrenchTorque(const JocobianMatrix& J) const
{
    const EEWrenchTarget target = desired_ee_wrench_->read();
    const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - target.stamp).count();
    if (age > config_.ee_wrench_timeout)
    {
        return JointTorque::Zero();
    }
    Eigen::Vector3d force = target.wrench.head<3>();
    Eigen::Vector3d torque = target.wrench.tail<3>();
    if (force.norm() > config_.max_ee_force)
    {
        force *= config_.max_ee_force / force.norm();
    }
    if (torque.norm() > config_.max_ee_torque)
    {
        torque *= config_.max_ee_torque / torque.norm();
    }
    transform::Vector6d wrench;
    wrench << force, torque;
    return J.transpose() * wrench;
}

franka::Torques HybridJointImpedanceControl::controlLoop(const franka::RobotState& robot_state,
                                                         franka::Duration duration)
{
    JointPosition target_pos;
    JointVelocity target_vel;
    jointTarget(desired_joint_command_->read(), config_.feedforward, target_pos, target_vel);
    // the controller tracks a jerk-limited reference, never the raw (possibly jumping) command
    JointPosition desired_pos;
    JointVelocity desired_vel;
    JointAcceleration desired_acc;
    reference_generator_.update(target_pos, target_vel, duration.toMSec(), desired_pos,
                                desired_vel, desired_acc);
    desired_acc *= config_.feedforward.acceleration_feedforward;
    const JointPosition current_pos = Eigen::Map<const JointPosition>(robot_state.q.data());
    const JointVelocity current_vel = Eigen::Map<const JointVelocity>(robot_state.dq.data());

    JocobianMatrix J = pinocchio_model_->computeJacobian(current_pos);
    Eigen::Matrix<double, 7, 7> Kp = (J.transpose() * config_.kx * J).eval() + config_.kq;
    Eigen::Matrix<double, 7, 7> Kd = (J.transpose() * config_.kxd * J).eval() + config_.kqd;

    auto joint_pos_error = desired_pos - current_pos;
    auto joint_vel_error = desired_vel - current_vel;

    Eigen::Matrix<double, 7, 1> torque_feedback = Kp * joint_pos_error + Kd * joint_vel_error;

    // M(q) * desired_acc + Coriolis; gravity is subtracted because libfranka already compensates it
    JointTorque torque_forward =
        pinocchio_model_->inverseDynamics(current_pos, current_vel, desired_acc);
    torque_forward -= pinocchio_model_->inverseDynamics(current_pos, JointVelocity::Zero(),
                                                        JointAcceleration::Zero());
    const JointTorque torque_wrench = eeWrenchTorque(J);
    std::array<double, 7> tau_cmd{};
    for (size_t i = 0; i < 7; i++)
    {
        tau_cmd[i] = torque_feedback[i] + torque_forward[i] + torque_wrench[i];
    }
    return franka::Torques{tau_cmd};
}
