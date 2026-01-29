#include "control_mode/abstract_control_mode.hpp"

#include <franka/command_types.h>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

void ControllerConfig::readBaseConfig(const ConfigFileReader& reader)
{
    controller_name = reader.getValue<std::string>("name");
    command_topic = reader.getValue<std::string>("command_topic");
}

void SafetyLimitConfig::fromFile(const std::string& controller_config_path)
{
    ConfigFileReader reader(controller_config_path);
    limit_rate = reader.getValue<bool>("limit_rate");
    lpf_cutoff_freq = reader.getValue<double>("lpf_cutoff_freq");
    cartesian_pos_upper_limits = reader.getArray<double, 3>("cartesian_pos_upper_limits");
    cartesian_pos_lower_limits = reader.getArray<double, 3>("cartesian_pos_lower_limits");
    joint_pos_upper_limits = reader.getArray<double, NUM_DOFS>("joint_pos_upper_limits");
    joint_pos_lower_limits = reader.getArray<double, NUM_DOFS>("joint_pos_lower_limits");
    const std::array<double, NUM_DOFS> joint_vel_limits =
        reader.getArray<double, NUM_DOFS>("joint_vel_limits");
    joint_vel_upper_limits = joint_vel_limits;
    joint_vel_lower_limits = std::array<double, NUM_DOFS>{};
    for (size_t i = 0; i < NUM_DOFS; i++)
    {
        joint_vel_lower_limits[i] = -joint_vel_limits[i];
    }
    joint_torques_limits = reader.getArray<double, NUM_DOFS>("joint_torques_limits");
    margin_joint_pos = reader.getValue<double>("margin_joint_pos");
    margin_joint_vel = reader.getValue<double>("margin_joint_vel");
    k_joint_pos = reader.getValue<double>("k_joint_pos");
    k_joint_vel = reader.getValue<double>("k_joint_vel");
}

void AbstractControlMode::initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                                         AtomicDoubleBuffer<franka::RobotState>& state_buffer)
{
    robot_ = &robot;
    pinocchio_model_ = &pinocchio_model;
    state_buffer_ = &state_buffer;
}

void AbstractControlMode::startControl()
{
    robot_->automaticErrorRecovery();
    zlc::info("[{}] Robot control started.", getModeName());
    is_running_ = true;
    control_thread_ = std::thread(&AbstractControlMode::controlTask, this);
    zlc::info("[{}] Control thread launched.", getModeName());
}

void AbstractControlMode::stopControl()
{
    is_running_ = false;
    if (control_thread_.joinable())
    {
        zlc::info("[{}] Stopping control thread...", getModeName());
        control_thread_.join();
    }
    zlc::info("[{}] Stopped.", getModeName());
}

const std::string AbstractControlMode::getModeName()
{
    return controller_name;
}

void AbstractControlMode::controlTask()
{
    zlc::info("[{}] Control thread started.", getModeName());
    auto control_callback = [this](const franka::RobotState& state,
                                   franka::Duration duration) -> franka::Torques
    { return this->controlLoop(state, duration); };
    while (is_running_)
    {
        try
        {
            robot_->control(control_callback);
        }
        catch (const std::exception& ex)
        {
            zlc::error("[CartesianVelocityMode] Robot is unable to be controlled: {}", ex.what());
            break;
        }
        bool recovered = tryRecovery();
        if (!recovered)
        {
            zlc::error("[CartesianVelocityMode] Unable to recover robot. Exiting control loop.");
            break;
        }
    }
    zlc::info("[{}] Control thread ended.", getModeName());
}
MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal)
        : q_goal_(q_goal.data()) {
    dq_max_ *= speed_factor;
    ddq_max_start_ *= speed_factor;
    ddq_max_goal_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
}
franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
    time_ += period.toSec();

    if (time_ == 0.0) {
        q_start_ = MotionGenerator::Vector7d(robot_state.q.data());
        delta_q_ = q_goal_ - q_start_;
        calculateSynchronizedValues();
    }

    MotionGenerator::Vector7d delta_q_d;
    bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(joint_positions.data(), 7) = (q_start_ + delta_q_d);
    franka::JointPositions output(joint_positions);
    output.motion_finished = motion_finished;
    return output;
}
bool MotionGenerator::calculateDesiredValues(double time, MotionGenerator::Vector7d* delta_q_d) const {
    MotionGenerator::Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();
    MotionGenerator::Vector7d t_d = t_2_sync_ - t_1_sync_;
    MotionGenerator::Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
    std::array<bool, 7> joint_motion_finished{};

    for (Eigen::Index i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
        (*delta_q_d)[i] = 0;
        joint_motion_finished[i] = true;
        } else {
        if (time < t_1_sync_[i]) {
            (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                            (0.5 * time - t_1_sync_[i]) * std::pow(time, 3.0);
        } else if (time >= t_1_sync_[i] && time < t_2_sync_[i]) {
            (*delta_q_d)[i] = q_1_[i] + (time - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
        } else if (time >= t_2_sync_[i] && time < t_f_sync_[i]) {
            (*delta_q_d)[i] =
                delta_q_[i] +
                0.5 *
                    (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                        (time - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                        std::pow((time - t_1_sync_[i] - t_d[i]), 3.0) +
                    (2.0 * time - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                    dq_max_sync_[i] * sign_delta_q[i];
        } else {
            (*delta_q_d)[i] = delta_q_[i];
            joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool is_finished) { return is_finished; });
}

void MotionGenerator::calculateSynchronizedValues() {
    MotionGenerator::Vector7d dq_max_reach(dq_max_);
    MotionGenerator::Vector7d t_f = MotionGenerator::Vector7d::Zero();
    MotionGenerator::Vector7d delta_t_2 = MotionGenerator::Vector7d::Zero();
    MotionGenerator::Vector7d t_1 = MotionGenerator::Vector7d::Zero();
    MotionGenerator::Vector7d delta_t_2_sync = MotionGenerator::Vector7d::Zero();
    MotionGenerator::Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();

    for (Eigen::Index i = 0; i < 7U; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                    3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
            dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                        (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                        (ddq_max_start_[i] + ddq_max_goal_[i]));
        }
        t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
        delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
        t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
        }
    }
    double max_t_f = t_f.maxCoeff();
    for (Eigen::Index i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
        double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);            // NOLINT
        double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];         // NOLINT
        double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];  // NOLINT
        double delta = b * b - 4.0 * a * c;
        if (delta < 0.0) {
            delta = 0.0;
        }
        dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
        delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
        t_f_sync_[i] =
            (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
        q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}
bool AbstractControlMode::moveToJointPosition(const std::array<double, NUM_DOFS>& target_q,
                                              double max_velocity, double tolerance)
{
#if NO_ROBOT_TESTING
    zlc::error("[{}] moveToJointPosition is not supported in NO_ROBOT_TESTING mode.",
               getModeName());
    return false;
#else
    if (!robot_)
    {
        zlc::error("[{}] moveToJointPosition failed: robot not initialized.", getModeName());
        return false;
    }
    if (is_running_)
    {
        zlc::warn("[{}] moveToJointPosition rejected: control thread is running.", getModeName());
        return false;
    }
    try {
        robot_->setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        MotionGenerator motion_generator(max_velocity, target_q);
        robot_->control(motion_generator);
    }catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
    return true;
#endif
}


bool AbstractControlMode::tryRecovery(int max_attempts)
{
    for (size_t i = 0; i < max_attempts; i++)
    {
        try
        {
            robot_->automaticErrorRecovery();
            zlc::info("[{}] Recovery successful.", getModeName());
            return true;
        }
        catch (const franka::Exception& e)
        {
            zlc::error("[{}] Recovery failed: {}", getModeName(), e.what());
            return false;
        }
    }
    return false;
}

void AbstractControlMode::checkStateLimits(const franka::RobotState& robot_state,
                                           franka::Torques& torque_out,
                                           const SafetyLimitConfig& safety_config_)
{
    /*
   * Compute robot state limit violations and apply safety mechanisms.
   */
    std::array<double, 3> ee_pos_buf, force_buf;
    std::array<double, 1> elbow_vel_buf, elbow_lim_buf, dummy;

    // Reset reflex torques
    for (int i = 0; i < 7; i++)
    {
        torque_out.tau_J[i] = 0.0;
    }
    for (int i = 0; i < 3; i++)
    {
        force_buf[i] = 0.0;
    }

    // Joint position limits
    computeSafetyReflex(robot_state.q, safety_config_.joint_pos_lower_limits,
                        safety_config_.joint_pos_upper_limits, torque_out.tau_J,
                        safety_config_.margin_joint_pos, safety_config_.k_joint_pos);

    // Joint velocity limits
    computeSafetyReflex(robot_state.dq, safety_config_.joint_vel_lower_limits,
                        safety_config_.joint_vel_upper_limits, torque_out.tau_J,
                        safety_config_.margin_joint_vel, safety_config_.k_joint_vel);

    for (int i = 0; i < 7; i++)
    {
        torque_out.tau_J[i] =
            std::clamp(torque_out.tau_J[i], -safety_config_.joint_torques_limits[i],
                       safety_config_.joint_torques_limits[i]);
    }
}

template <std::size_t N>
void AbstractControlMode::computeSafetyReflex(std::array<double, N> values,
                                              std::array<double, N> lower_limit,
                                              std::array<double, N> upper_limit,
                                              std::array<double, N>& torques_out, double margin,
                                              double k)
{
    /*
   * Apply safety mechanisms for a vector based on input values and limits.
   * Throws an error if limits are violated.
   * Also computes & outputs safety controller torques.
   * (Note: invert_lower flips the sign of the lower limit. Used for velocities
   * and torques.)
   */
    double upper_violation, lower_violation;

    // Check limits & compute safety controller
    for (int i = 0; i < N; i++)
    {
        upper_violation = values[i] - upper_limit[i];
        lower_violation = lower_limit[i] - values[i];
        if (upper_violation > 0 || lower_violation > 0)
        {
            zlc::warn(
                "Safety limit violated on index {}: value = {}, lower limit = {}, upper limit = {}",
                i, values[i], lower_limit[i], upper_limit[i]);
        }
        if (upper_violation > -margin)
        {
            torques_out[i] -= k * (margin + upper_violation);
        }
        else if (lower_violation > -margin)
        {
            torques_out[i] += k * (margin + lower_violation);
        }
    }
}