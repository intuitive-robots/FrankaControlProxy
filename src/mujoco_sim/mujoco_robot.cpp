#include "mujoco_sim/mujoco_robot.hpp"

#include <franka/exception.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include "mujoco_sim/mujoco_panda_env.hpp"

namespace
{
    constexpr std::chrono::milliseconds kControlPeriod{1}; // 1 kHz

    franka::Duration toDuration(std::chrono::nanoseconds dt)
    {
        return franka::Duration(static_cast<uint64_t>(dt.count()));
    }
} // namespace

MujocoRobot::MujocoRobot(const std::string&)
    : env_(std::make_unique<MujocoPandaEnv>("./models/franka_emika_panda/scene.xml")),
      viewer_(std::make_unique<MujocoViewer>(env_.get()))
{
    env_->start();
    env_->refreshRobotState(current_state_);
    viewer_->start();
}

MujocoRobot::~MujocoRobot() noexcept
{
    stop();
}

void MujocoRobot::control(
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback,
    bool /*limit_rate*/, double /*cutoff_frequency*/)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::control: A control loop is already running.");
    }

    running_ = true;
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    auto last_tick = next_tick;

    while (running_)
    {
        const auto now = clock::now();
        auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_tick);
        last_tick = now;
        franka::Torques torques = control_callback(current_state_, toDuration(dt_ns));
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_.tau_J_d = torques.tau_J;
        }
        env_->nextStep(torques, current_state_);
        if (torques.motion_finished)
        {
            running_ = false;
            break;
        }

        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }
}

void MujocoRobot::read(std::function<bool(const franka::RobotState&)> read_callback)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::read: A control loop is already running.");
    }

    running_ = true;
    auto next_tick = std::chrono::steady_clock::now();
    franka::Torques zero_torques{};
    zero_torques.tau_J.fill(0.0);

    while (running_)
    {
        if (!read_callback(current_state_))
        {
            running_ = false;
            break;
        }
        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }
}

franka::RobotState MujocoRobot::readOnce()
{
    if (running_)
    {
        throw franka::InvalidOperationException(
            "MujocoRobot::readOnce: A control loop is already running.");
    }
    return current_state_;
}

void MujocoRobot::setCollisionBehavior(
    const std::array<double, 7>& /*lower_torque_thresholds_acceleration*/,
    const std::array<double, 7>& /*upper_torque_thresholds_acceleration*/,
    const std::array<double, 7>& /*lower_torque_thresholds_nominal*/,
    const std::array<double, 7>& /*upper_torque_thresholds_nominal*/,
    const std::array<double, 6>& /*lower_force_thresholds_acceleration*/,
    const std::array<double, 6>& /*upper_force_thresholds_acceleration*/,
    const std::array<double, 6>& /*lower_force_thresholds_nominal*/,
    const std::array<double, 6>& /*upper_force_thresholds_nominal*/)
{
    // No-op: collision behavior is not modeled in this MuJoCo wrapper.
}

void MujocoRobot::setCollisionBehavior(const std::array<double, 7>& /*lower_torque_thresholds*/,
                                       const std::array<double, 7>& /*upper_torque_thresholds*/,
                                       const std::array<double, 6>& /*lower_force_thresholds*/,
                                       const std::array<double, 6>& /*upper_force_thresholds*/)
{
    // No-op: collision behavior is not modeled in this MuJoCo wrapper.
}

void MujocoRobot::automaticErrorRecovery()
{
    // No-op for simulation.
}

void MujocoRobot::stop()
{
    running_ = false;
    env_->stop();
    viewer_->stop();
}
