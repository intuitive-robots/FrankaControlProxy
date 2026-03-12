#include "control_mode/abstract_control_mode.hpp"

#include <franka/command_types.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "motion_generator/cartesian_pose_motion_generator.hpp"
#include "motion_generator/joint_position_motion_generator.hpp"

void ControllerConfig::readBaseConfig(const ConfigFileReader& reader)
{
    controller_name = reader.getValue<std::string>("name");
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
                                         AtomicDoubleBuffer<franka::RobotState>& state_buffer,
                                         const SafetyLimitConfig& safety_config)
{
    robot_ = &robot;
    pinocchio_model_ = &pinocchio_model;
    state_buffer_ = &state_buffer;
    safety_config_ = &safety_config;
    zlc::info("[{}] controller initialized.", getModeName());
}

void AbstractControlMode::startControl()
{
    robot_->automaticErrorRecovery();
    zlc::debug("[{}] control started.", getModeName());
    is_running_ = true;
    control_thread_ = std::thread(&AbstractControlMode::controlTask, this);
    zlc::debug("[{}] control thread launched.", getModeName());
}

void AbstractControlMode::stopControl()
{
    is_running_ = false;
    if (control_thread_.joinable())
    {
        zlc::debug("[{}] Stopping control thread...", getModeName());
        control_thread_.join();
    }
    zlc::info("[{}] Mode Stopped.", getModeName());
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
    {
        franka::Torques tau = this->controlLoop(state, duration);
        if (!is_running_)
        {
            return franka::MotionFinished(tau);
        }
        return tau;
    };

    while (is_running_)
    {
        try
        {
            robot_->control(control_callback);
            break;
        }
        catch (const std::exception& ex)
        {
            zlc::error("[{}] Robot is unable to be controlled: {}", getModeName(), ex.what());
        }
        bool recovered = tryRecovery();
        if (!recovered)
        {
            zlc::error("[{}] Unable to recover robot. Exiting control loop.", getModeName());
            break;
        }
    }
    zlc::info("[{}] Control thread ended.", getModeName());
}

bool AbstractControlMode::moveToJointPosition(const std::array<double, NUM_DOFS>& target_q,
                                              double max_velocity, double tolerance)
{
    stopControl();
    zlc::info("[{}] Moving to joint position ...", getModeName());
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
    for (size_t i = 0; i < 5; i++)
    {
        try
        {
            robot_->automaticErrorRecovery();
            JointPositionMotionGenerator motion_generator(max_velocity, target_q, *state_buffer_,
                                                          tolerance);
            robot_->control(motion_generator);
            break;
        }
        catch (const franka::Exception& e)
        {
            zlc::error("[{}] Error when move joint position: {}", getModeName(), e.what());
        }
        bool recovered = tryRecovery();
        if (!recovered)
        {
            zlc::error("[{}] Unable to recover robot. Exiting control loop.", getModeName());
            break;
        }
    }
    zlc::info("[{}] Reached target joint position.", getModeName());
    startControl();
    return true;
}

bool AbstractControlMode::moveToCartesianPose(const Eigen::Vector3d& target_position,
                                              const Eigen::Quaterniond& target_orientation,
                                              double max_velocity, double tolerance)
{
    if (!robot_)
    {
        zlc::error("[{}] moveToCartesianPose failed: robot not initialized.", getModeName());
        return false;
    }
    if (is_running_)
    {
        zlc::warn("[{}] moveToCartesianPose rejected: control thread is running.", getModeName());
        return false;
    }
    try
    {
        robot_->automaticErrorRecovery();
        CartesianPoseMotionGenerator motion_generator(
            max_velocity, target_position, target_orientation, *state_buffer_, tolerance);
        robot_->control(motion_generator);
    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

bool AbstractControlMode::tryRecovery(int max_attempts)
{
    for (size_t i = 0; i < max_attempts; i++)
    {
        try
        {
            robot_->automaticErrorRecovery();
            zlc::info("[{}] Recovery successful.", getModeName());
            break;
        }
        catch (const franka::Exception& e)
        {
            zlc::error("[{}] Recovery failed: {}", getModeName(), e.what());
            return false;
        }
    }
    return true;
}

void AbstractControlMode::checkStateLimits(const franka::RobotState& robot_state,
                                           franka::Torques& torque_out,
                                           const SafetyLimitConfig& safety_config_)
{
    /*
   * Compute robot state limit violations and apply safety mechanisms.
   */
    std::array<double, 3> force_buf;

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