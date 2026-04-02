#include "control_mode/hybrid_joint_impedance_control.hpp"

#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>

namespace
{
std::string formatJointCommand(const std::array<double, 7>& pos)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << "[";
    for (size_t i = 0; i < pos.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << pos[i];
    }
    oss << "]";
    return oss.str();
}
}  // namespace

HybridJointImpedanceControl::~HybridJointImpedanceControl() = default;

void HybridJointImpedanceControl::initController(
    FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
    AtomicDoubleBuffer<franka::RobotState>& state_buffer)
{
    AbstractControlMode::initController(robot, pinocchio_model, state_buffer);
    config_.fromFile("config/controller/hybrid_joint_impedance_controller.cfg");
    controller_name = config_.controller_name;
    std::array<double, 7> current_pos = state_buffer.read().q;
    desired_positions_.write(JointPosition::Map(current_pos.data()));
    auto pose = desired_positions_.read();
    std::cout << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << ","
              << pose[4] << "," << pose[5] << "," << pose[6] << std::endl;
    zlc::registerSubscriberHandler(config_.command_topic,
                                   &HybridJointImpedanceControl::writeCommand, this);
}

void HybridJointImpedanceControl::writeCommand(const HybridJointImpedanceCommand& cmd)
{
    zlc::info("Received command on '{}': {}", config_.command_topic, formatJointCommand(cmd.pos));
    desired_positions_.write(JointPosition::Map(cmd.pos.data()));
}

franka::Torques HybridJointImpedanceControl::controlLoop(const franka::RobotState& robot_state,
                                                         franka::Duration /*duration*/)
{
    state_buffer_->write(robot_state);
    const JointPosition desired_pos = desired_positions_.read();
    const JointPosition current_pos = Eigen::Map<const JointPosition>(robot_state.q.data());
    const JointVelocity desired_vel = JointVelocity::Zero();
    const JointVelocity current_vel = Eigen::Map<const JointVelocity>(robot_state.dq.data());

    JocobianMatrix J = pinocchio_model_->computeJacobian(current_pos);
    Eigen::Matrix<double, 7, 7> Kp = (J.transpose() * config_.kx * J).eval() + config_.kq;
    Eigen::Matrix<double, 7, 7> Kd = (J.transpose() * config_.kxd * J).eval() + config_.kqd;

    auto joint_pos_error = desired_pos - current_pos;
    auto joint_vel_error = desired_vel - current_vel;

    Eigen::Matrix<double, 7, 1> torque_feedback = Kp * joint_pos_error + Kd * joint_vel_error;

    JointTorque torque_forward =
        pinocchio_model_->inverseDynamics(current_pos, current_vel, JointAcceleration::Zero());
    torque_forward -= pinocchio_model_->inverseDynamics(
        current_pos, JointVelocity::Zero(), JointAcceleration::Zero());  // remove gravity compensation if needed
    std::array<double, 7> tau_cmd{};
    for (size_t i = 0; i < 7; i++)
    {
        tau_cmd[i] = torque_feedback[i] + torque_forward[i];
    }
    franka::Torques tau_command = franka::Torques{tau_cmd};
    if (!is_running_)
    {
        tau_command.motion_finished = true;
    }
    return tau_command;
}
